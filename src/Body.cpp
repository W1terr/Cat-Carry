#include "Body.h"

#include "MathUtil.h"

namespace Body
{
	namespace
	{
		using M::Cross;
		using M::Dot;
		using M::Length;
		using M::Normalized;
		using RE::NiAVObject;
		using RE::NiMatrix3;
		using RE::NiPoint3;
		using RE::NiPointer;
		using RE::NiTransform;

		// palm center in hand bone space (vanilla skeleton: fingers along +Z, middle finger base 8.3 out, palm faces -Y)
		const NiPoint3 kPalmCenter{ 0.0f, -1.4f, 4.4f };
		const NiPoint3 kHandFingers{ 0.0f, 0.0f, 1.0f };
		const NiPoint3 kHandPalm{ 0.0f, -1.0f, 0.0f };
		// elbow bend normal (upper arm direction x forearm direction) in the upper arm's and forearm's own space:
		// the vanilla forearm sits on the upper arm rotated about local X only
		const NiPoint3 kDefaultHinge{ -1.0f, 0.0f, 0.0f };

		constexpr float kMaxWrist = 85.0f * M::kDegrees;
		constexpr float kMaxHeadTurn = 70.0f * M::kDegrees;
		constexpr float kNeckShare = 0.4f;
		constexpr float kMaxCurl = 1.1f;  // radians per finger joint at full curl
		constexpr float kReachMargin = 0.93f;
		constexpr std::array kSpineShare{ 0.3f, 0.35f, 0.35f };
		constexpr std::array kTwistShare{ 0.66f, 0.33f };  // ForearmTwist1 sits near the wrist, Twist2 halfway up

		struct Joint
		{
			NiPointer<NiAVObject> node;
			NiMatrix3             base;     // the animated local rotation this frame
			NiMatrix3             written;  // what we wrote last
			bool                  wrote{ false };

			explicit operator bool() const { return node != nullptr; }

			// puts the animated rotation back if the animation didn't overwrite ours; true if it did that
			bool Begin()
			{
				if (!node) {
					return false;
				}
				bool restored = false;
				if (wrote && M::Same(node->local.rotate, written)) {
					node->local.rotate = base;
					restored = true;
				} else {
					base = node->local.rotate;
				}
				wrote = false;
				return restored;
			}

			void Write(const NiMatrix3& a_local)
			{
				node->local.rotate = a_local;
				written = a_local;
				wrote = true;
			}

			void WriteWorld(const NiMatrix3& a_world)
			{
				const NiMatrix3 parent = node->parent ? node->parent->world.rotate : M::Identity();
				Write(parent.Transpose() * a_world);
			}

			void Update()
			{
				RE::NiUpdateData context{};
				node->Update(context);
			}

			bool Release()
			{
				bool restored = false;
				if (node && wrote && M::Same(node->local.rotate, written)) {
					node->local.rotate = base;
					restored = true;
				}
				wrote = false;
				node.reset();
				return restored;
			}
		};

		struct Finger
		{
			Joint    joint;
			NiPoint3 axis;  // local axis that curls the finger towards the palm
			float    share{ 1.0f };
		};

		struct Arm
		{
			Joint               upper;
			Joint               fore;
			Joint               hand;
			std::array<Joint, 2> twist;
			std::vector<Finger> fingers;
			NiPoint3            hingeUpper{ kDefaultHinge };
			NiPoint3            hingeFore{ kDefaultHinge };
			std::array<NiTransform, 3> animated;  // upper, forearm, hand world before anything of ours
			float               twistAngle{ 0.0f };
			NiPoint3            foreAxis{ 0.0f, 0.0f, 1.0f };
		};

		// skin bone slots without a node (finger / twist bones the skeleton lacks): the game points them at some
		// transform that follows the animated arm, so they get our own transform that follows the posed arm
		struct Redirect
		{
			NiPointer<RE::NiSkinInstance> skin;
			std::uint32_t                 index{ 0 };
			const NiTransform*            original{ nullptr };
			std::unique_ptr<NiTransform>  own;
			int                           arm{ 0 };
			int                           bone{ 0 };  // 0 upper arm, 1 forearm, 2 hand
			float                         twist{ 0.0f };
			NiTransform                   originalValue;
			std::optional<NiTransform>    fixed;  // no game transform: kept relative to the arm bone like this
		};

		struct State
		{
			NiPointer<NiAVObject>  root;
			std::array<Joint, 3>   spine;
			Joint                  neck;
			Joint                  head;
			std::array<Arm, 2>     arms;
			std::vector<Redirect>  redirects;
			std::array<NiPoint3, 3> spinePos;
			NiMatrix3               chestRot;
			NiPoint3               headPos;
			bool                   ready{ false };
		} state;

		NiAVObject* Find(NiAVObject* a_root, const std::string& a_name)
		{
			return a_root ? a_root->GetObjectByName(a_name) : nullptr;
		}

		NiAVObject* Ancestor(NiAVObject* a_node, const std::string& a_name)
		{
			for (auto* node = a_node ? a_node->parent : nullptr; node; node = node->parent) {
				if (node->name == a_name.c_str()) {
					return node;
				}
			}
			return nullptr;
		}

		float SegmentDistance(const NiPoint3& p, const NiPoint3& a, const NiPoint3& b, float& a_t)
		{
			const NiPoint3 ab = b - a;
			const float    length = Dot(ab, ab);
			a_t = length > 1e-6f ? std::clamp(Dot(p - a, ab) / length, 0.0f, 1.0f) : 0.0f;
			return Length(p - (a + ab * a_t));
		}

		bool IsArmBone(const NiAVObject* a_node)
		{
			for (auto& arm : state.arms) {
				if (a_node == arm.upper.node.get() || a_node == arm.fore.node.get() || a_node == arm.hand.node.get()) {
					return true;
				}
			}
			return false;
		}

		void FindRedirects()
		{
			std::size_t meshes = 0;
			RE::BSVisit::TraverseScenegraphGeometries(state.root.get(), [&](RE::BSGeometry* a_geometry) {
				auto* skin = a_geometry->GetGeometryRuntimeData().skinInstance.get();
				if (!skin || !skin->skinData || !skin->bones || !skin->boneWorldTransforms) {
					return RE::BSVisit::BSVisitControl::kContinue;
				}
				const std::uint32_t count = skin->skinData->GetBoneCount();
				bool                armMesh = false;
				for (std::uint32_t i = 0; i < count && !armMesh; ++i) {
					armMesh = skin->bones[i] && IsArmBone(skin->bones[i]);
				}
				if (!armMesh) {
					return RE::BSVisit::BSVisitControl::kContinue;
				}
				// Each slot without a node follows the arm bone of the slot nearest to it in the mesh's own rest pose
				// (picking by where the game shows the slot missed fingertips, which sit further than 9 out from the
				// hand bone, and they stretched).
				std::size_t found = 0;
				std::size_t missing = 0;
				auto*       data = skin->skinData.get();
				for (std::uint32_t i = 0; i < count; ++i) {
					if (skin->bones[i]) {
						continue;
					}
					++missing;
					const NiPoint3 bind = data->GetBoneDataSkinToBone(i).Invert().translate;
					std::uint32_t  nearest = count;
					float          best = 1e30f;
					for (std::uint32_t j = 0; j < count; ++j) {
						if (!skin->bones[j]) {
							continue;
						}
						const float distance = Length(data->GetBoneDataSkinToBone(j).Invert().translate - bind);
						if (distance < best) {
							best = distance;
							nearest = j;
						}
					}
					if (nearest == count) {
						continue;
					}
					// the arm bone it belongs to: the nearest slot's bone or one of its parents
					int bestArm = -1;
					int bestBone = 0;
					for (auto* node = skin->bones[nearest]; node && bestArm < 0; node = node->parent) {
						for (int arm = 0; arm < 2 && bestArm < 0; ++arm) {
							const auto& chain = state.arms[arm];
							const std::array<const NiAVObject*, 3> bones{ chain.upper.node.get(), chain.fore.node.get(), chain.hand.node.get() };
							for (int k = 0; k < 3; ++k) {
								if (node == bones[k]) {
									bestArm = arm;
									bestBone = k;
								}
							}
						}
					}
					if (bestArm < 0) {
						continue;
					}
					Redirect redirect;
					redirect.skin.reset(skin);
					redirect.index = i;
					redirect.original = skin->boneWorldTransforms[i];
					redirect.arm = bestArm;
					redirect.bone = bestBone;
					redirect.twist = bestBone == 1 ? 0.5f : 0.0f;
					if (redirect.original) {
						redirect.originalValue = *redirect.original;
					} else {
						// no transform from the game: the rest pose relation to the arm bone
						const auto& arm = state.arms[bestArm];
						const auto* bone = bestBone == 0 ? arm.upper.node.get() : (bestBone == 1 ? arm.fore.node.get() : arm.hand.node.get());
						redirect.originalValue = skin->bones[nearest]->world * (data->GetBoneDataSkinToBone(nearest) * data->GetBoneDataSkinToBone(i).Invert());
						redirect.fixed = bone->world.Invert() * redirect.originalValue;
					}
					redirect.own = std::make_unique<NiTransform>(redirect.originalValue);
					state.redirects.push_back(std::move(redirect));
					++found;
				}
				if (missing > found) {
					logs::info("Mesh {}: {} of {} bones without a node aren't on an arm", a_geometry->name.c_str(), missing - found, missing);
				}
				if (found) {
					++meshes;
					logs::info("Mesh {} has {} bone(s) without a node, they follow the posed arm", a_geometry->name.c_str(), found);
				}
				return RE::BSVisit::BSVisitControl::kContinue;
			});
			if (meshes) {
				logs::info("{} skin bone(s) in {} mesh(es) follow the posed arms", state.redirects.size(), meshes);
			}
		}

		void DropRedirects()
		{
			for (auto& redirect : state.redirects) {
				if (redirect.skin && redirect.skin->boneWorldTransforms &&
					redirect.skin->boneWorldTransforms[redirect.index] == redirect.own.get()) {
					redirect.skin->boneWorldTransforms[redirect.index] = redirect.original;
				}
			}
			state.redirects.clear();
		}

		bool SetupArm(Side a_side)
		{
			auto&      arm = state.arms[a_side];
			const char s = a_side == kLeft ? 'L' : 'R';
			auto*      hand = Find(state.root.get(), std::format("NPC {} Hand [{}Hnd]", s, s));
			auto*      fore = Ancestor(hand, std::format("NPC {} Forearm [{}Lar]", s, s));
			auto*      upper = Ancestor(fore, std::format("NPC {} UpperArm [{}Uar]", s, s));
			if (!hand || !fore || !upper) {
				logs::warn("The {} arm of the player's skeleton wasn't found", a_side == kLeft ? "left" : "right");
				return false;
			}
			arm.hand.node.reset(hand);
			arm.fore.node.reset(fore);
			arm.upper.node.reset(upper);
			for (int i = 0; i < 2; ++i) {
				arm.twist[i].node.reset(Find(fore, std::format("NPC {} ForearmTwist{} [{}Lt{}]", s, i + 1, s, i + 1)));
			}

			// elbow hinge from the animated arm when it's bent enough to tell, else the vanilla skeleton's
			const NiPoint3 u = fore->world.translate - upper->world.translate;
			const NiPoint3 f = hand->world.translate - fore->world.translate;
			const NiPoint3 bend = Cross(u, f);
			if (Length(bend) > std::sin(12.0f * M::kDegrees) * Length(u) * Length(f)) {
				const NiPoint3 n = Normalized(bend);
				arm.hingeUpper = Normalized(upper->world.rotate.Transpose() * n);
				arm.hingeFore = Normalized(fore->world.rotate.Transpose() * n);
			}

			arm.fingers.clear();
			const NiPoint3 palm = hand->world.rotate * kHandPalm;
			for (int finger = 0; finger < 5; ++finger) {
				NiAVObject* previous = hand;
				for (int joint = 0; joint < 3; ++joint) {
					auto* node = Find(hand, std::format("NPC {} Finger{}{} [{}F{}{}]", s, finger, joint, s, finger, joint));
					if (!node) {
						continue;
					}
					// the finger's direction from where the joints are: XPMSSE puts "CME" nodes between the finger bones,
					// so a child's local offset isn't in this bone's space (fingers bent sideways / backwards)
					auto*          child = Find(hand, std::format("NPC {} Finger{}{} [{}F{}{}]", s, finger, joint + 1, s, finger, joint + 1));
					const NiPoint3 worldAlong = child ? child->world.translate - node->world.translate : node->world.translate - previous->world.translate;
					previous = node;
					NiPoint3 along = Length(worldAlong) > 1e-3f ? Normalized(node->world.rotate.Transpose() * worldAlong) : kHandFingers;
					NiPoint3 axis = Cross(along, node->world.rotate.Transpose() * palm);
					if (Length(axis) < 1e-3f) {
						continue;
					}
					Finger entry;
					entry.joint.node.reset(node);
					entry.axis = Normalized(axis);
					entry.share = (finger == 0 ? 0.45f : 1.0f) * (joint == 1 ? 1.0f : 0.8f);
					arm.fingers.push_back(std::move(entry));
				}
			}
			return true;
		}

		template <class F>
		void ForEachJoint(F&& a_func)
		{
			for (auto& joint : state.spine) {
				a_func(joint);
			}
			a_func(state.neck);
			a_func(state.head);
			for (auto& arm : state.arms) {
				a_func(arm.upper);
				a_func(arm.fore);
				a_func(arm.hand);
				for (auto& joint : arm.twist) {
					a_func(joint);
				}
				for (auto& finger : arm.fingers) {
					a_func(finger.joint);
				}
			}
		}

		void UpdateRoot()
		{
			if (state.root) {
				RE::NiUpdateData context{};
				state.root->Update(context);
			}
		}

		// a point carried along by the spine lean
		NiPoint3 LeanPoint(const NiPoint3& a_point, float a_lean, const NiPoint3& a_right)
		{
			NiPoint3                point = a_point;
			std::array<NiPoint3, 3> pivots = state.spinePos;
			for (std::size_t j = 0; j < 3; ++j) {
				if (!state.spine[j]) {
					continue;
				}
				const NiMatrix3 turn = M::AxisAngle(a_right, a_lean * kSpineShare[j]);
				const NiPoint3  pivot = pivots[j];
				point = pivot + turn * (point - pivot);
				for (std::size_t k = j + 1; k < 3; ++k) {
					pivots[k] = pivot + turn * (pivots[k] - pivot);
				}
			}
			return point;
		}

		NiPoint3 HandTarget(const HandGoal& a_goal, const Arm& a_arm)
		{
			return a_goal.palm - a_goal.rotation * (kPalmCenter * a_arm.hand.node->world.scale);
		}

		void Lean(const Goals& a_goals)
		{
			if (std::abs(a_goals.lean) < 1e-4f) {
				return;
			}
			for (std::size_t j = 0; j < 3; ++j) {
				auto& joint = state.spine[j];
				if (!joint) {
					continue;
				}
				joint.WriteWorld(M::AxisAngle(a_goals.right, a_goals.lean * kSpineShare[j]) * joint.node->world.rotate);
				joint.Update();
			}
		}

		void Look(const Goals& a_goals)
		{
			if (!a_goals.lookAt || a_goals.lookWeight <= 0.0f || !state.head) {
				return;
			}
			auto turnTowards = [&](Joint& a_joint, float a_amount) {
				const NiPoint3 face = state.head.node->world.rotate * NiPoint3{ 0.0f, 1.0f, 0.0f };
				const NiPoint3 want = Normalized(*a_goals.lookAt - state.head.node->world.translate, face);
				const NiMatrix3 turn = M::Slerp(M::Identity(), M::Limit(M::Between(face, want), kMaxHeadTurn), a_amount);
				a_joint.WriteWorld(turn * a_joint.node->world.rotate);
				a_joint.Update();
			};
			const float weight = M::Clamp01(a_goals.lookWeight);
			if (state.neck) {
				turnTowards(state.neck, weight * kNeckShare);
			}
			turnTowards(state.head, weight);
		}

		// a fixed turn shared by the neck and the head
		void TurnHead(const Goals& a_goals)
		{
			if (!a_goals.headTurn || !state.head) {
				return;
			}
			const NiMatrix3& turn = *a_goals.headTurn;
			float            done = 0.0f;
			if (state.neck) {
				state.neck.WriteWorld(M::Slerp(M::Identity(), turn, kNeckShare) * state.neck.node->world.rotate);
				state.neck.Update();
				done = kNeckShare;
			}
			// the head already turned with the neck: the rest of the way
			state.head.WriteWorld(M::Slerp(M::Identity(), turn, 1.0f - done) * state.head.node->world.rotate);
			state.head.Update();
		}

		// the hand straight on from the forearm, its palm facing where the goal's palm faces: where the forearm will
		// point depends on the hand (the wrist target is behind the palm), so twice
		HandGoal StraightWrist(const HandGoal& a_goal, const Arm& a_arm)
		{
			HandGoal       goal = a_goal;
			const NiPoint3 palmFacing = a_goal.rotation * NiPoint3{ 0.0f, -1.0f, 0.0f };
			const NiPoint3 s = a_arm.upper.node->world.translate;
			const float    a = Length(a_arm.fore.node->world.translate - s);
			const float    b = Length(a_arm.hand.node->world.translate - a_arm.fore.node->world.translate);
			if (a < 1e-3f || b < 1e-3f) {
				return goal;
			}
			for (int pass = 0; pass < 2; ++pass) {
				const NiPoint3 target = HandTarget(goal, a_arm);
				const NiPoint3 dir = Normalized(target - s);
				const float    dist = std::clamp(Length(target - s), std::abs(a - b) + 0.5f, (a + b) * 0.995f);
				const NiPoint3 pole = Normalized(M::Perpendicular(a_goal.pole, dir), { 0.0f, 0.0f, -1.0f });
				const float    cosine = std::clamp((a * a + dist * dist - b * b) / (2.0f * a * dist), -1.0f, 1.0f);
				const float    sine = std::sqrt(1.0f - cosine * cosine);
				const NiPoint3 elbow = s + (dir * cosine + pole * sine) * a;
				const NiPoint3 forearm = Normalized(s + dir * dist - elbow, dir);
				goal.rotation = M::Align({ 0.0f, 0.0f, 1.0f }, { 0.0f, -1.0f, 0.0f }, forearm, palmFacing);
			}
			return goal;
		}

		void SolveArm(Arm& a_arm, const HandGoal& a_wanted)
		{
			a_arm.twistAngle = 0.0f;
			const float w = M::Clamp01(a_wanted.weight);
			if (w <= 0.0f) {
				return;
			}
			const HandGoal a_goal = a_wanted.straight ? StraightWrist(a_wanted, a_arm) : a_wanted;
			const NiPoint3  s = a_arm.upper.node->world.translate;
			const NiPoint3  e0 = a_arm.fore.node->world.translate;
			const NiPoint3  h0 = a_arm.hand.node->world.translate;
			const NiMatrix3 upperRot = a_arm.upper.node->world.rotate;
			const NiMatrix3 foreRot = a_arm.fore.node->world.rotate;
			const NiMatrix3 handRot = a_arm.hand.node->world.rotate;
			const float     a = Length(e0 - s);
			const float     b = Length(h0 - e0);
			if (a < 1e-3f || b < 1e-3f) {
				return;
			}

			// wrist position, inside the arm's reach
			NiPoint3       t = M::Lerp(h0, HandTarget(a_goal, a_arm), w);
			const NiPoint3 dir = Normalized(t - s);
			const float    dist = std::clamp(Length(t - s), std::abs(a - b) + 0.5f, (a + b) * 0.995f);
			t = s + dir * dist;

			// elbow: on the side of the pole, blended from where the animation had it
			const NiPoint3 animPole = Normalized(M::Perpendicular(e0 - s, dir), { 0.0f, 0.0f, -1.0f });
			const NiPoint3 goalPole = Normalized(M::Perpendicular(a_goal.pole, dir), animPole);
			const NiPoint3 pole = Normalized(M::Lerp(animPole, goalPole, w), animPole);
			const float    cosine = std::clamp((a * a + dist * dist - b * b) / (2.0f * a * dist), -1.0f, 1.0f);
			const float    sine = std::sqrt(1.0f - cosine * cosine);
			const NiPoint3 e = s + (dir * cosine + pole * sine) * a;

			NiPoint3 normal = Cross(e - s, t - e);
			normal = Length(normal) > 1e-4f * a * b ? Normalized(normal) : Normalized(upperRot * a_arm.hingeUpper);

			// upper arm and forearm turn about the elbow hinge only
			const NiPoint3 upperAlong = Normalized(upperRot.Transpose() * (e0 - s));
			a_arm.upper.WriteWorld(M::Align(upperAlong, a_arm.hingeUpper, Normalized(e - s), normal));
			a_arm.upper.Update();

			const NiPoint3 elbow = a_arm.fore.node->world.translate;
			const NiPoint3 foreAlong = Normalized(foreRot.Transpose() * (h0 - e0));
			const NiMatrix3 foreNew = M::Align(foreAlong, a_arm.hingeFore, Normalized(t - elbow), normal);
			a_arm.fore.WriteWorld(foreNew);
			a_arm.fore.Update();

			// hand: from the animated wrist bend towards the goal rotation, within what a wrist can do
			const NiMatrix3 relAnimated = foreRot.Transpose() * handRot;
			const NiMatrix3 follow = foreNew * relAnimated;
			const NiMatrix3 change = M::Limit(a_goal.rotation * follow.Transpose(), kMaxWrist);
			const NiMatrix3 handNew = M::Slerp(M::Identity(), change, w) * follow;
			a_arm.hand.WriteWorld(handNew);
			a_arm.hand.Update();

			// part of the wrist twist goes up the forearm
			const NiMatrix3 twist = (foreNew.Transpose() * handNew) * relAnimated.Transpose();
			a_arm.twistAngle = M::Twist(twist, foreAlong);
			a_arm.foreAxis = Normalized(t - elbow);
			for (std::size_t i = 0; i < 2; ++i) {
				auto& joint = a_arm.twist[i];
				if (joint) {
					joint.WriteWorld(M::AxisAngle(a_arm.foreAxis, a_arm.twistAngle * kTwistShare[i]) * joint.node->world.rotate);
					joint.Update();
				}
			}

			const float curl = M::Clamp01(a_goal.curl) * w * kMaxCurl;
			if (curl > 0.0f && !a_arm.fingers.empty()) {
				for (auto& finger : a_arm.fingers) {
					finger.joint.Write(finger.joint.node->local.rotate * M::AxisAngle(finger.axis, curl * finger.share));
				}
				a_arm.hand.Update();
			}
		}

		void MoveRedirects()
		{
			for (auto& redirect : state.redirects) {
				if (!redirect.skin || !redirect.skin->boneWorldTransforms) {
					continue;
				}
				const auto& arm = state.arms[redirect.arm];
				const auto& node = redirect.bone == 0 ? arm.upper : (redirect.bone == 1 ? arm.fore : arm.hand);
				const auto& animated = arm.animated[redirect.bone];
				NiTransform own = node.node->world * (redirect.fixed ? *redirect.fixed : animated.Invert() * redirect.originalValue);
				if (redirect.twist > 0.0f && arm.twistAngle != 0.0f) {
					own.rotate = M::AxisAngle(arm.foreAxis, arm.twistAngle * redirect.twist) * own.rotate;
				}
				*redirect.own = own;
				redirect.skin->boneWorldTransforms[redirect.index] = redirect.own.get();
			}
		}
	}

	bool Setup(RE::Actor* a_player)
	{
		Release();
		auto* root = a_player ? a_player->Get3D(false) : nullptr;
		if (!root) {
			return false;
		}
		state.root.reset(root);
		if (!SetupArm(kLeft) || !SetupArm(kRight)) {
			Release();
			return false;
		}
		const std::array<const char*, 3> spine{ "NPC Spine [Spn0]", "NPC Spine1 [Spn1]", "NPC Spine2 [Spn2]" };
		for (std::size_t i = 0; i < 3; ++i) {
			state.spine[i].node.reset(Find(root, spine[i]));
		}
		state.neck.node.reset(Find(root, "NPC Neck [Neck]"));
		state.head.node.reset(Find(root, "NPC Head [Head]"));
		FindRedirects();
		state.ready = true;
		logs::info("Player skeleton ready: fingers {}/{}, forearm twist bones {}/{}, spine bones {}",
			state.arms[0].fingers.size(), state.arms[1].fingers.size(),
			(state.arms[0].twist[0] ? 1 : 0) + (state.arms[0].twist[1] ? 1 : 0),
			(state.arms[1].twist[0] ? 1 : 0) + (state.arms[1].twist[1] ? 1 : 0),
			std::ranges::count_if(state.spine, [](const Joint& a_joint) { return static_cast<bool>(a_joint); }));
		return true;
	}

	void Release()
	{
		DropRedirects();
		bool restored = false;
		ForEachJoint([&](Joint& a_joint) { restored |= a_joint.Release(); });
		for (auto& arm : state.arms) {
			arm.fingers.clear();
		}
		if (restored) {
			UpdateRoot();
		}
		state.root.reset();
		state.ready = false;
	}

	bool Ready()
	{
		return state.ready;
	}

	void BeginFrame()
	{
		if (!state.ready) {
			return;
		}
		bool restored = false;
		ForEachJoint([&](Joint& a_joint) { restored |= a_joint.Begin(); });
		if (restored) {
			UpdateRoot();
		}
		for (auto& arm : state.arms) {
			arm.animated = { arm.upper.node->world, arm.fore.node->world, arm.hand.node->world };
			arm.twistAngle = 0.0f;
		}
		for (std::size_t i = 0; i < 3; ++i) {
			state.spinePos[i] = state.spine[i] ? state.spine[i].node->world.translate : NiPoint3{};
		}
		state.headPos = state.head ? state.head.node->world.translate : NiPoint3{};
		state.chestRot = state.spine[2] ? state.spine[2].node->world.rotate : M::Identity();
		for (auto& redirect : state.redirects) {
			if (redirect.original && !redirect.fixed) {
				redirect.originalValue = *redirect.original;
			}
		}
	}

	void Apply(const Goals& a_goals)
	{
		if (!state.ready) {
			return;
		}
		Lean(a_goals);
		TurnHead(a_goals);
		Look(a_goals);
		for (int side = 0; side < 2; ++side) {
			SolveArm(state.arms[side], a_goals.hands[side]);
		}
		MoveRedirects();
	}

	NiPoint3 Shoulder(Side a_side)
	{
		return state.arms[a_side].animated[0].translate;
	}

	NiPoint3 Spine2()
	{
		if (state.spine[2]) {
			return state.spinePos[2];
		}
		return (Shoulder(kLeft) + Shoulder(kRight)) * 0.5f - NiPoint3{ 0.0f, 0.0f, 15.0f };
	}

	std::optional<NiMatrix3> ChestRotation()
	{
		if (!state.ready || !state.spine[2]) {
			return std::nullopt;
		}
		return state.chestRot;
	}

	NiPoint3 Head()
	{
		return state.head ? state.headPos : Spine2() + NiPoint3{ 0.0f, 0.0f, 29.0f };
	}

	float ArmLength(Side a_side)
	{
		const auto& animated = state.arms[a_side].animated;
		return Length(animated[1].translate - animated[0].translate) + Length(animated[2].translate - animated[1].translate);
	}

	float LeanFor(const Goals& a_goals, float a_max)
	{
		if (!state.ready) {
			return 0.0f;
		}
		constexpr float kStep = 2.0f * M::kDegrees;
		for (float lean = 0.0f; lean < a_max; lean += kStep) {
			bool reaches = true;
			for (int side = 0; side < 2 && reaches; ++side) {
				const auto& goal = a_goals.hands[side];
				if (goal.weight <= 0.01f) {
					continue;
				}
				const NiPoint3 shoulder = LeanPoint(Shoulder(static_cast<Side>(side)), lean, a_goals.right);
				reaches = Length(HandTarget(goal, state.arms[side]) - shoulder) <= ArmLength(static_cast<Side>(side)) * kReachMargin;
			}
			if (reaches) {
				return lean;
			}
		}
		return a_max;
	}
}
