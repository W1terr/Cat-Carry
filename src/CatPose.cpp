#include "CatPose.h"

#include "MathUtil.h"

#include <unordered_map>

namespace CatPose
{
	namespace
	{
		using M::Normalized;
		using RE::NiAVObject;
		using RE::NiMatrix3;
		using RE::NiPoint3;
		using RE::NiPointer;
		using RE::NiTransform;

		// one bone turned so the direction to its child points somewhere
		struct Aim
		{
			NiPointer<NiAVObject> bone;
			NiPointer<NiAVObject> child;
			std::optional<NiPoint3> point;   // instead of a child: a point in the bone's own space (toes of a paw)
			NiMatrix3             captured;  // local rotation when we took the cat
			float                 weight{ 1.0f };
			float                 phase{ 0.0f };  // left and right legs move out of step
		};

		enum class Chain
		{
			kHindLeg,
			kFrontUpper,  // shoulder to elbow: forward over your arm
			kFrontLower,  // elbow to paw: hangs down
			kBreath,      // chest rises and falls
			kPaw,         // front paw: wrist to fingers, hangs
			kFoot,        // hind foot: ankle to toes, hangs
			kTail,
			kNeck,
			kHead,
			kJaw,  // cow skeleton (Piglet)
			kEar
		};

		struct Posed
		{
			Chain chain;
			Aim   aim;
		};

		struct Collision
		{
			NiPointer<NiAVObject> node;
			RE::COL_LAYER         layer;
		};

		struct State
		{
			NiPointer<NiAVObject> root;
			// The cat's skeleton in game is a cut-down tree (27 bones: no fingers, toes, ears, tongue); the mesh's slots
			// for the missing bones have no node and point at some fixed transform, so paws and toes stayed behind when
			// the legs moved. Each such slot gets its own transform following its bone if the skeleton has it after
			// all, else the bone nearest to it in the bind pose.
			struct Follower
			{
				NiPointer<RE::NiSkinInstance> skin;
				std::uint32_t                 index{ 0 };
				const NiTransform*            original{ nullptr };
				NiPointer<NiAVObject>         follow;
				NiTransform                   offset;  // follow bone world -> this slot's world (from the bind pose)
				std::unique_ptr<NiTransform>  own;
			};
			std::vector<Follower> followers;
			// its physics bodies, taken out of the world while it's in your arms (inside your capsule they pushed you)
			std::vector<NiPointer<RE::bhkWorldObject>> bodies;
			NiPointer<RE::bhkWorld>                    world;
			std::vector<Collision> collision;  // layers before we made the held cat non-colliding (parents first)
			std::vector<Posed>    bones;
			NiPoint3              nose{ 0.0f, 1.0f, 0.0f };  // HEAD -> nose in HEAD space
			NiPoint3              headUp{ 0.0f, 0.0f, 1.0f };  // up of the standing animal in HEAD space
			float                 scale{ 1.0f };             // the root's own local scale (it's smaller in your arms)
			Frame                 frame;
		} state;

		// the skeleton's bones: the cat's mesh brings its own copies of every bone (under its "Scene Root"), which the
		// name search may find first
		NiAVObject* Skeleton(NiAVObject* a_root)
		{
			auto* skeleton = a_root ? a_root->GetObjectByName("NPC Root [Root]") : nullptr;
			return skeleton ? skeleton : a_root;
		}

		NiPoint3 ToRoot(const NiAVObject* a_root, const NiPoint3& a_world);

		NiAVObject* Bone(NiAVObject* a_root, const char* a_name)
		{
			auto* skeleton = Skeleton(a_root);
			return skeleton ? skeleton->GetObjectByName(a_name) : nullptr;
		}

		// the chest: "Torso" on the cats' skeever skeleton, "Ribcage" on the cow skeleton (Piglet)
		NiAVObject* Chest(NiAVObject* a_root)
		{
			auto* bone = Bone(a_root, "Torso");
			return bone ? bone : Bone(a_root, "Ribcage");
		}

		NiAVObject* HeadBone(NiAVObject* a_root)
		{
			auto* bone = Bone(a_root, "HEAD");
			return bone ? bone : Bone(a_root, "Scull");
		}

		bool Inside(const NiAVObject* a_node, const NiAVObject* a_root)
		{
			for (auto* node = a_node; node; node = node->parent) {
				if (node == a_root) {
					return true;
				}
			}
			return false;
		}

		NiPoint3 ToRoot(const NiAVObject* a_root, const NiPoint3& a_world)
		{
			const auto& w = a_root->world;
			return (w.rotate.Transpose() * (a_world - w.translate)) * (1.0f / w.scale);
		}

		void Add(NiAVObject* a_root, Chain a_chain, const char* a_bone, const char* a_child, float a_weight)
		{
			auto* bone = Bone(a_root, a_bone);
			auto* child = Bone(a_root, a_child);
			if (!bone || !child) {
				return;
			}
			Posed posed{ a_chain, {} };
			posed.aim.bone.reset(bone);
			posed.aim.child.reset(child);
			posed.aim.captured = bone->local.rotate;
			posed.aim.weight = a_weight;
			posed.aim.phase = a_bone[0] == 'R' ? M::kPi : 0.0f;
			state.bones.push_back(std::move(posed));
		}

		void Update(NiAVObject* a_node)
		{
			RE::NiUpdateData context{};
			a_node->Update(context);
		}

		// The pose is worked out on paper first (our own forward kinematics), then every bone is written once and the
		// cat updated once, so nothing reading the skeleton meanwhile sees it half way between two poses.
		struct Solver
		{
			NiTransform                                      root;
			std::unordered_map<const NiAVObject*, NiMatrix3> locals;  // posed bones' local rotation this frame

			NiTransform Local(const NiAVObject* a_node) const
			{
				NiTransform local = a_node->local;
				if (auto it = locals.find(a_node); it != locals.end()) {
					local.rotate = it->second;
				}
				return local;
			}

			NiTransform World(const NiAVObject* a_node) const
			{
				if (a_node == state.root.get() || !a_node->parent) {
					return root;
				}
				return World(a_node->parent) * Local(a_node);
			}

			// turns a bone by a world rotation
			void Turn(const NiAVObject* a_bone, const NiMatrix3& a_worldTurn)
			{
				const NiMatrix3 world = World(a_bone).rotate;
				const NiMatrix3 parent = World(a_bone->parent).rotate;
				locals[a_bone] = parent.Transpose() * (a_worldTurn * world);
			}

			void AimAt(const Aim& a_aim, const NiPoint3& a_direction, float a_amount)
			{
				const NiTransform bone = World(a_aim.bone.get());
				const NiPoint3    now = a_aim.point ? bone.rotate * *a_aim.point : World(a_aim.child.get()).translate - bone.translate;
				if (M::Length(now) < 1e-4f || a_amount <= 0.0f) {
					return;
				}
				Turn(a_aim.bone.get(), M::Slerp(M::Identity(), M::Between(now, a_direction), M::Clamp01(a_amount)));
			}
		};
	}

	Frame Measure(NiAVObject* a_root)
	{
		Frame frame;
		auto* torso = Chest(a_root);
		auto* pelvis = Bone(a_root, "Pelvis");
		if (torso && pelvis) {
			frame.torso = ToRoot(a_root, torso->world.translate);
			frame.pelvis = ToRoot(a_root, pelvis->world.translate);
		} else {
			// unknown skeleton: along the model's forward axis through its bound
			const float    reach = a_root->worldBound.radius / a_root->world.scale * 0.35f;
			const NiPoint3 center = ToRoot(a_root, a_root->worldBound.center);
			frame.torso = center + NiPoint3{ 0.0f, reach, 0.0f };
			frame.pelvis = center - NiPoint3{ 0.0f, reach, 0.0f };
		}
		frame.middle = (frame.torso + frame.pelvis) * 0.5f;
		frame.spine = Normalized(frame.torso - frame.pelvis, { 0.0f, 1.0f, 0.0f });
		frame.back = Normalized(M::Perpendicular({ 0.0f, 0.0f, 1.0f }, frame.spine), { 0.0f, 0.0f, 1.0f });
		frame.side = M::Cross(frame.spine, frame.back);
		frame.halfLength = M::Length(frame.torso - frame.pelvis) * 0.5f;
		return frame;
	}

	bool Hold(RE::Actor* a_cat)
	{
		Release();
		auto* root = a_cat ? a_cat->Get3D() : nullptr;
		if (!root || !root->parent) {
			return false;
		}
		state.root.reset(root);
		state.scale = root->local.scale;
		state.frame = Measure(root);
		// its body collision rides along on its bones, right inside your own capsule: off while it's held
		RE::BSVisit::TraverseScenegraphObjects(root, [](NiAVObject* a_object) {
			if (a_object->collisionObject) {
				state.collision.push_back({ NiPointer<NiAVObject>(a_object), a_object->GetCollisionLayer() });
			}
			return RE::BSVisit::BSVisitControl::kContinue;
		});
		root->SetCollisionLayer(RE::COL_LAYER::kNonCollidable);
		if (auto* cell = a_cat->GetParentCell(); cell) {
			state.world.reset(cell->GetbhkWorld());
		}
		if (state.world) {
			RE::BSVisit::TraverseScenegraphCollision(root, [](RE::bhkNiCollisionObject* a_collision) {
				auto* body = a_collision->body.get();
				if (body && body->GetWorld1()) {
					state.bodies.emplace_back(body);
					body->MoveToWorld(nullptr);
				}
				return RE::BSVisit::BSVisitControl::kContinue;
			});
		}

		auto*       skeleton = Skeleton(root);
		std::size_t slots = 0;
		RE::BSVisit::TraverseScenegraphGeometries(root, [&](RE::BSGeometry* a_geometry) {
			auto* skin = a_geometry->GetGeometryRuntimeData().skinInstance.get();
			if (!skin || !skin->skinData || !skin->bones || !skin->boneWorldTransforms) {
				return RE::BSVisit::BSVisitControl::kContinue;
			}
			auto*               data = skin->skinData.get();
			const std::uint32_t count = data->GetBoneCount();
			slots += count;
			for (std::uint32_t i = 0; i < count; ++i) {
				if (skin->bones[i]) {
					continue;
				}
				// bind pose of the missing bone in skin space, and the nearest slot that has a bone
				const NiTransform bind = data->GetBoneDataSkinToBone(i).Invert();
				std::uint32_t     nearest = count;
				float             best = 1e30f;
				for (std::uint32_t j = 0; j < count; ++j) {
					if (!skin->bones[j]) {
						continue;
					}
					const float distance = M::Length(data->GetBoneDataSkinToBone(j).Invert().translate - bind.translate);
					if (distance < best) {
						best = distance;
						nearest = j;
					}
				}
				if (nearest == count) {
					continue;
				}
				decltype(state.followers)::value_type follower;
				follower.skin.reset(skin);
				follower.index = i;
				follower.original = skin->boneWorldTransforms[i];
				follower.follow.reset(skin->bones[nearest]);
				// Keep the slot where the game showed it on the floor, relative to the followed bone (the rest pose
				// offset bent the toes and fingers into hooks: the standing animation turns ankles and wrists away
				// from the rest pose). The rest pose offset only when the game's transform is missing.
				const NiTransform restOffset = data->GetBoneDataSkinToBone(nearest) * bind;
				// The skeleton may have the slot's bone after all, only not linked to the skin (Piglet's head: the
				// slot sits exactly on "Scull" but had no bone, so it followed Neck3 and didn't turn with the head).
				// A skeleton node right where the game shows the slot is that bone: follow it.
				NiAVObject* same = nullptr;
				if (follower.original) {
					float closest = 0.5f * std::max(root->world.scale, 0.1f);
					RE::BSVisit::TraverseScenegraphObjects(skeleton, [&](NiAVObject* a_node) {
						const float distance = M::Length(a_node->world.translate - follower.original->translate);
						if (a_node->AsNode() && distance < closest &&
							M::Angle(a_node->world.rotate.Transpose() * follower.original->rotate) < 10.0f * M::kDegrees) {
							closest = distance;
							same = a_node;
						}
						return RE::BSVisit::BSVisitControl::kContinue;
					});
				}
				if (same) {
					follower.follow.reset(same);
				}
				follower.offset = follower.original ? follower.follow->world.Invert() * *follower.original : restOffset;
				follower.own = std::make_unique<NiTransform>(follower.follow->world * follower.offset);
				skin->boneWorldTransforms[i] = follower.own.get();
				state.followers.push_back(std::move(follower));
			}
			return RE::BSVisit::BSVisitControl::kContinue;
		});
		logs::info("Skin: {} bone slots, {} without a bone in the skeleton follow one", slots, state.followers.size());
		for (const char* side : { "L", "R" }) {
			// skeever skeleton (House Cats)
			const std::string leg = std::format("{}Leg", side);
			Add(root, Chain::kHindLeg, (leg + "1").c_str(), (leg + "2").c_str(), 0.6f);
			Add(root, Chain::kHindLeg, (leg + "2").c_str(), (leg + "3").c_str(), 0.4f);
			Add(root, Chain::kHindLeg, (leg + "3").c_str(), (leg + "Ankle").c_str(), 0.3f);
			const std::string arm = std::format("{}Arm_", side);
			Add(root, Chain::kFrontUpper, (arm + "Upper").c_str(), (arm + "Forearm").c_str(), 1.0f);
			Add(root, Chain::kFrontLower, (arm + "Forearm").c_str(), (arm + "Palm").c_str(), 0.9f);
			// cow skeleton (Piglet): femur, tibia, cannon bone, then the hoof; humerus, radius, metacarpus, then the hoof
			const std::string hind = side;
			Add(root, Chain::kHindLeg, (hind + "Femur").c_str(), (hind + "Tibia").c_str(), 0.6f);
			Add(root, Chain::kHindLeg, (hind + "Tibia").c_str(), (hind + "LargeCannon").c_str(), 0.4f);
			Add(root, Chain::kHindLeg, (hind + "LargeCannon").c_str(), (hind + "PhalanxPrima").c_str(), 0.3f);
			Add(root, Chain::kFoot, (hind + "PhalanxPrima").c_str(), (hind + "RearHoof").c_str(), 0.8f);
			const std::string front = std::format("{}_", side);
			Add(root, Chain::kFrontUpper, (front + "Humerus").c_str(), (front + "Radius").c_str(), 1.0f);
			Add(root, Chain::kFrontLower, (front + "Radius").c_str(), (front + "Metacarpus").c_str(), 0.9f);
			Add(root, Chain::kPaw, (front + "Metacarpus").c_str(), (front + "PhalangesManus").c_str(), 0.7f);
		}
		Add(root, Chain::kTail, "Tail1", "Tail2", 0.55f);
		Add(root, Chain::kTail, "Tail2", "Tail3", 0.6f);
		Add(root, Chain::kTail, "Tail3", "Tail4", 0.65f);
		Add(root, Chain::kNeck, "Neck", "HEAD", 0.35f);
		Add(root, Chain::kNeck, "Neck1", "Scull", 0.35f);
		// The skeleton has no finger / toe bones: the paws are turned towards where their fingers and toes are (the
		// slots that follow the palm / ankle), so they hang with the leg instead of sticking out flat like hooks.
		for (const auto& [name, chain, weight] : { std::tuple{ "LArm_Palm", Chain::kPaw, 0.7f }, std::tuple{ "RArm_Palm", Chain::kPaw, 0.7f },
				 std::tuple{ "LLegAnkle", Chain::kFoot, 0.8f }, std::tuple{ "RLegAnkle", Chain::kFoot, 0.8f } }) {
			auto* bone = Bone(root, name);
			if (!bone) {
				continue;
			}
			NiPoint3    sum{};
			std::size_t count = 0;
			for (const auto& follower : state.followers) {
				if (follower.follow.get() == bone) {
					sum += follower.offset.translate;
					++count;
				}
			}
			if (!count || M::Length(sum) < 1e-3f) {
				continue;
			}
			Posed posed{ chain, {} };
			posed.aim.bone.reset(bone);
			posed.aim.point = sum * (1.0f / static_cast<float>(count));
			posed.aim.captured = bone->local.rotate;
			posed.aim.weight = weight;
			state.bones.push_back(std::move(posed));
		}
		if (auto* head = HeadBone(root); head) {
			std::optional<NiPoint3> nose;
			auto*                   lip = Bone(root, "UpperLipFront");
			auto*                   leftMuzzle = Bone(root, "Scull_[Lmuzzel]");
			auto*                   rightMuzzle = Bone(root, "Scull_[Rmuzzel]");
			if (lip) {
				nose = lip->world.translate;
			} else if (leftMuzzle && rightMuzzle) {
				nose = (leftMuzzle->world.translate + rightMuzzle->world.translate) * 0.5f;
			}
			if (nose) {
				state.nose = Normalized(head->world.rotate.Transpose() * (*nose - head->world.translate), state.nose);
			} else {
				// the cats' skeleton in game has no lip bone: the standing cat faces forward and a bit down
				state.nose = Normalized(head->world.rotate.Transpose() * (root->world.rotate * NiPoint3{ 0.0f, 1.0f, -0.15f }), state.nose);
			}
			Posed posed{ Chain::kHead, {} };
			posed.aim.bone.reset(head);
			posed.aim.captured = head->local.rotate;
			posed.aim.weight = 0.85f;
			state.bones.push_back(std::move(posed));
			state.headUp = Normalized(head->world.rotate.Transpose() * (root->world.rotate * NiPoint3{ 0.0f, 0.0f, 1.0f }), state.headUp);
			// after the head (they're its children): jaw and ears of the cow skeleton
			for (const auto& [name, chain] : { std::pair{ "Jaw", Chain::kJaw }, std::pair{ "Scull_[Lear]", Chain::kEar }, std::pair{ "Scull_[Rear]", Chain::kEar } }) {
				if (auto* bone = Bone(root, name); bone) {
					Posed part{ chain, {} };
					part.aim.bone.reset(bone);
					part.aim.captured = bone->local.rotate;
					part.aim.phase = name[6] == 'R' ? 1.0f : 0.0f;
					state.bones.push_back(std::move(part));
				}
			}
		}
		if (auto* torso = Chest(root); torso) {
			Posed posed{ Chain::kBreath, {} };
			posed.aim.bone.reset(torso);
			posed.aim.captured = torso->local.rotate;
			state.bones.insert(state.bones.begin(), std::move(posed));
		}
		logs::info("Holding {} ({} posed bones)", a_cat->GetDisplayFullName(), state.bones.size());
		return true;
	}

	void Release()
	{
		if (state.root) {
			// SetCollisionLayer sets a whole subtree: parents first, then each child gets its own back
			for (const auto& entry : state.collision) {
				entry.node->SetCollisionLayer(entry.layer);
			}
			for (const auto& body : state.bodies) {
				if (state.world && !body->GetWorld1()) {
					body->MoveToWorld(state.world.get());
				}
			}
			for (const auto& follower : state.followers) {
				if (follower.skin && follower.skin->boneWorldTransforms &&
					follower.skin->boneWorldTransforms[follower.index] == follower.own.get()) {
					follower.skin->boneWorldTransforms[follower.index] = follower.original;
				}
			}
			for (const auto& posed : state.bones) {
				posed.aim.bone->local.rotate = posed.aim.captured;
			}
			state.root->local.scale = state.scale;
			if (state.root->parent) {
				Update(state.root.get());
			}
		}
		state.bones.clear();
		state.collision.clear();
		state.followers.clear();
		state.bodies.clear();
		state.world.reset();
		state.root.reset();
	}

	bool Active()
	{
		return state.root != nullptr;
	}

	NiAVObject* Root()
	{
		return state.root.get();
	}

	void Place(const NiTransform& a_world, float a_hold, float a_time, const NiPoint3& a_look, float a_sway, float a_pawsForward, float a_legs,
		const Mood& a_mood)
	{
		auto* root = state.root.get();
		if (!root || !root->parent) {
			return;
		}
		Solver solver;
		solver.root = a_world;
		for (const auto& posed : state.bones) {
			solver.locals[posed.aim.bone.get()] = posed.aim.captured;
		}
		const float      hold = M::Clamp01(a_hold);
		const NiMatrix3& rotation = a_world.rotate;
		const NiPoint3   down{ 0.0f, 0.0f, -1.0f };
		const Frame&     body = state.frame;
		const NiPoint3 belly = rotation * (body.back * -1.0f);
		const NiPoint3 side = rotation * body.side;
		// standing, its hind legs point to its belly side; held upright that's forward: let them hang only half way down
		// (a full turn at the hips stretched its belly)
		const NiPoint3 hind = Normalized(down + belly * 0.9f);
		// front legs over your forearm like over a branch: upper leg forward, the rest hangs
		const NiPoint3 frontUpper = Normalized(belly * a_pawsForward + down * 0.45f);
		const NiPoint3 frontLower = Normalized(down + belly * 0.15f);
		const NiPoint3 paw = Normalized(down - belly * 0.1f);
		const NiPoint3 foot = Normalized(down + belly * 0.2f);
		const float    sway = (0.45f * std::sin(a_time * 1.1f) + 0.15f * std::sin(a_time * 2.7f)) * a_sway +
		                   0.55f * std::sin(a_time * 17.0f) * M::Clamp01(a_mood.tailWag);  // a happy wiggle
		// running in the air: a swing forward / back along the belly, diagonal legs together, lower legs a bit late
		const float    paddle = M::Clamp01(a_mood.paddle) * 0.75f;
		auto           run = [&](const Aim& a_aim, bool a_front, float a_lag) {
			const float phase = a_aim.phase + (a_front ? M::kPi : 0.0f);
			return belly * (paddle * std::sin(a_time * 11.0f + phase - a_lag));
		};
		const float    flick = 0.3f * std::pow(std::max(0.0f, std::sin(a_time * 0.45f + 1.0f)), 6.0f) * a_sway;  // now and then
		const NiPoint3 tail = Normalized(down + belly * (0.15f + flick) + side * sway);

		for (const auto& posed : state.bones) {
			const bool  leg = posed.chain == Chain::kHindLeg || posed.chain == Chain::kFrontUpper || posed.chain == Chain::kFrontLower ||
			                 posed.chain == Chain::kPaw || posed.chain == Chain::kFoot;
			const float amount = posed.aim.weight * hold * (leg ? M::Clamp01(a_legs) : 1.0f);
			if (amount <= 0.0f) {
				continue;
			}
			switch (posed.chain) {
			case Chain::kHindLeg:
				solver.AimAt(posed.aim, Normalized(hind + run(posed.aim, false, 0.0f)), amount);
				break;
			case Chain::kFrontUpper:
				solver.AimAt(posed.aim, Normalized(frontUpper + run(posed.aim, true, 0.0f)), amount);
				break;
			case Chain::kFrontLower:
				// a slow little kneading, left and right out of step
				solver.AimAt(posed.aim, Normalized(frontLower + belly * (0.12f * std::sin(a_time * 1.6f + posed.aim.phase)) + run(posed.aim, true, 0.8f)), amount);
				break;
			case Chain::kBreath:
				solver.Turn(posed.aim.bone.get(), M::AxisAngle(side, 2.0f * M::kDegrees * std::sin(a_time * 2.4f) * hold));
				break;
			case Chain::kPaw:
				solver.AimAt(posed.aim, paw, amount);
				break;
			case Chain::kFoot:
				solver.AimAt(posed.aim, foot, amount);
				break;
			case Chain::kTail:
				solver.AimAt(posed.aim, tail, amount);
				break;
			case Chain::kNeck:
				// the neck lifts the head up out of the body line
				solver.AimAt(posed.aim, Normalized(M::Lerp(rotation * body.spine, a_look, 0.35f) + NiPoint3{ 0.0f, 0.0f, 0.6f }),
					amount * std::max(a_mood.neck, 0.0f));
				break;
			case Chain::kHead:
				{
					const NiPoint3 nose = solver.World(posed.aim.bone.get()).rotate * state.nose;
					solver.Turn(posed.aim.bone.get(), M::Slerp(M::Identity(), M::Limit(M::Between(nose, a_look), 80.0f * M::kDegrees), M::Clamp01(amount)));
					// sniffing: quick little nods of the snout
					if (a_mood.sniff > 0.0f) {
						const NiMatrix3 head = solver.World(posed.aim.bone.get()).rotate;
						const NiPoint3  across = Normalized(M::Cross(head * state.headUp, head * state.nose));
						solver.Turn(posed.aim.bone.get(), M::AxisAngle(across, 5.0f * M::kDegrees * std::sin(a_time * 38.0f) * a_mood.sniff * hold));
					}
					// the animal's own head settings: up / down, to the side, tilted
					if (a_mood.headPitch != 0.0f || a_mood.headYaw != 0.0f || a_mood.headRoll != 0.0f) {
						const NiMatrix3 head = solver.World(posed.aim.bone.get()).rotate;
						const NiPoint3  up = Normalized(head * state.headUp);
						const NiPoint3  snout = Normalized(head * state.nose);
						const NiPoint3  across = Normalized(M::Cross(up, snout));
						// + about across turns the snout down: up is - pitch
						const NiMatrix3 turn = M::AxisAngle(snout, a_mood.headRoll * hold) * M::AxisAngle(up, a_mood.headYaw * hold) *
						                       M::AxisAngle(across, -a_mood.headPitch * hold);
						solver.Turn(posed.aim.bone.get(), turn);
					}
				}
				break;
			case Chain::kJaw:
				{
					// the mouth works while it sniffs: the snout tip turns towards the chin
					const NiMatrix3 head = solver.World(posed.aim.bone.get()->parent).rotate;
					const NiPoint3  open = Normalized(M::Cross(head * state.nose, head * state.headUp * -1.0f));
					const float     angle = 7.0f * M::kDegrees * (0.5f + 0.5f * std::sin(a_time * 19.0f)) * M::Clamp01(a_mood.sniff) * hold;
					solver.Turn(posed.aim.bone.get(), M::AxisAngle(open, angle));
				}
				break;
			case Chain::kEar:
				{
					// forward = the ear tips turn from up towards the snout; a twitch of its own now and then
					const NiMatrix3 head = solver.World(posed.aim.bone.get()->parent).rotate;
					const NiPoint3  forward = Normalized(M::Cross(head * state.headUp, head * state.nose));
					const float     twitch = std::pow(std::max(0.0f, std::sin(a_time * (1.9f + 0.6f * posed.aim.phase) + 3.0f * posed.aim.phase)), 30.0f);
					const float     angle = (25.0f * std::clamp(a_mood.ears, -1.0f, 1.0f) + 18.0f * twitch * std::sin(a_time * 30.0f)) * M::kDegrees * hold;
					solver.Turn(posed.aim.bone.get(), M::AxisAngle(forward, angle));
				}
				break;
			}
		}
		// all bones once, then one update
		for (const auto& posed : state.bones) {
			posed.aim.bone->local.rotate = solver.locals[posed.aim.bone.get()];
		}
		root->local = M::ToLocal(root->parent->world, a_world);
		Update(root);
		for (const auto& follower : state.followers) {
			*follower.own = follower.follow->world * follower.offset;
		}
	}
}
