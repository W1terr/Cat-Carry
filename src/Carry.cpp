#include "Carry.h"

#include "Body.h"
#include "CatPose.h"
#include "MathUtil.h"
#include "Settings.h"

#include <atomic>
#include <filesystem>
#include <fstream>

namespace Carry
{
	namespace
	{
		using M::Normalized;
		using RE::NiMatrix3;
		using RE::NiPoint3;
		using RE::NiTransform;
		using UEFlag = RE::ControlMap::UEFlag;

		// timings (seconds)
		constexpr float kTurnTime = 0.35f;
		constexpr float kSlideTime = 0.6f;
		constexpr float kKneelReachStart = 0.3f;
		constexpr float kKneelReachTime = 1.05f;
		constexpr float kStandReachTime = 0.7f;
		constexpr float kGrabTime = 0.2f;
		constexpr float kLiftTime = 1.0f;
		constexpr float kLowerTime = 1.2f;
		constexpr float kLowerStart = 0.25f;
		constexpr float kLetGoTime = 0.35f;
		constexpr float kStandTime = 1.15f;  // the kneel exit clip is 1.0 s
		constexpr float kSheatheTimeout = 2.5f;
		constexpr float kAppearTimeout = 2.0f;
		constexpr float kQuickLand = 0.25f;
		constexpr float kQuickTime = 0.4f;
		constexpr float kParkBehind = 150.0f;   // the held cat's actor (its capsule) waits this far behind you
		constexpr float kParkAgain = 3000.0f;   // ... and is brought along when you get this far from it
		constexpr float kFollowEvery = 1.0f;

		// where the cat's body ends up in front of you while kneeling (distance from your feet)
		constexpr float kReachDistance = 36.0f;
		constexpr float kMinDistance = 26.0f;
		constexpr float kPoseLength = 22.0f;  // a House Cat's chest to hips: the hand settings are made for it
		constexpr float kPoseDepth = 6.0f;    // ... and its spine to the middle of its body, back to belly

		constexpr float kMaxSlide = 80.0f;
		constexpr float kBaseLean = 6.0f * M::kDegrees;
		constexpr float kLeanRate = 6.0f;
		constexpr float kFlankAngle = 115.0f * M::kDegrees;

		// petting: your chest hand strokes the cat twice, then holds it again
		constexpr float kPetTime = 3.4f;
		constexpr int   kStrokes = 2;
		constexpr float kPetBlend = 0.45f;

		constexpr auto kKneelEnter = "IdleKneelingEnter";
		constexpr auto kKneelExit = "IdleChairExitStart";
		constexpr auto kDefaultState = "IdleForceDefaultState";

		enum class Phase
		{
			kIdle,
			kSheathe,  // waiting for the weapon to go away
			kReach,    // turn, step closer, kneel, hands to the cat
			kGrab,
			kLift,
			kHold,
			kLower,    // kneel, cat down to the floor
			kLetGo,    // the real cat comes back, hands leave it
			kStand,
			kQuickDrop  // attacked: the cat goes down beside you at once, hands free to fight
		};

		// where a hand touches the cat, around its spine (cat root space)
		struct Contact
		{
			bool      torso{ true };  // around its chest, else around its hips
			float     angle{ 0.0f };  // around the spine: 0 its back, +90 its right side, 180 its belly
			float     radius{ 5.0f };
			float     along{ 0.0f };  // along the spine from the anchor
			NiMatrix3 hand;           // hand bone rotation in cat root space
		};

		struct BodyFrame
		{
			NiPoint3 origin;
			NiPoint3 right;
			NiPoint3 forward;
			NiPoint3 up{ 0.0f, 0.0f, 1.0f };

			NiPoint3 Point(float x, float y, float z) const { return origin + right * x + forward * y + up * z; }
			NiPoint3 Direction(float x, float y, float z) const { return right * x + forward * y + up * z; }
		};

		struct State
		{
			Phase           phase{ Phase::kIdle };
			float           time{ 0.0f };
			float           clock{ 0.0f };
			RE::ActorHandle cat;
			bool            kneeling{ false };
			bool            mirrored{ false };  // held on your right side
			bool            wasFirstPerson{ false };
			bool            catHidden{ false };   // the cat is in your arms: frozen, its actor parked behind you
			RE::NiAVObject* catRoot{ nullptr };   // the 3D being held (compared only: a reload means a new one)
			float           followTimer{ 0.0f };
			bool            catAI{ true };
			bool            catBack{ false };  // the real cat was put back into the world
			bool            putDownRequest{ false };
			bool            keysBlocked{ false };  // sprint, sneak and POV keys
			bool            chestKnown{ false };
			NiMatrix3       chestToBody;            // body frame axes in the chest bone's space
			float           health{ -1.0f };
			NiTransform     dropFrom;
			UEFlag          disabled{ UEFlag::kNone };
			float           catScale{ 1.0f };  // the cat's own size (world scale on the floor)
			float           animalSize{ 1.0f };   // from its animal file (CatCarry\Animals\*.ini)
			float           animalGirth{ 1.0f };
			float           animalBelly{ 0.0f };
			bool            piglet{ false };
			std::size_t     head{ std::string::npos };  // index into heads (npos: an animal without a file)
			// picking up / putting down: you stay on your spot facing the cat, the camera doesn't turn you
			bool            locked{ false };
			float           lockHeading{ 0.0f };
			NiPoint3        lockPos;
			float           drivenFor{ 0.0f };  // seconds the behavior graph drove you by itself while holding the cat

			NiPoint3 startPos;
			NiPoint3 standPos;
			float    startHeading{ 0.0f };
			float    targetHeading{ 0.0f };

			CatPose::Frame         frame;
			NiTransform            ground;  // cat root on the floor (where it was picked up / where it goes)
			float                  dropHeading{ 0.0f };
			std::array<Contact, 2> floorContact;
			std::array<Contact, 2> holdContact;
			float                  lean{ 0.0f };
			float                  look{ 0.0f };
		} state;

		// the animals you can pick up: CatCarry\Animals\*.ini (FOMOD options) and sRaces of CatCarry.ini
		struct Animal
		{
			RE::TESRace* race{ nullptr };
			std::string  name;
			float        size{ 1.0f };   // times the size settings in your arms
			float        girth{ 1.0f };  // how round it is: hands this much further from its spine
			float        belly{ 0.0f };  // its body's middle this far below the spine bones (House Cat sized units)
			bool         piglet{ false };  // sStyle = piglet: its own little show in your arms
			std::size_t  head{ std::string::npos };  // its file's head settings in heads
		};
		std::vector<Animal>     animals;
		std::vector<AnimalHead> heads;

		Head CurrentHead()
		{
			return state.head < heads.size() ? heads[state.head].head : Head{};
		}

		const Animal* FindAnimal(const RE::TESRace* a_race)
		{
			const auto it = std::ranges::find(animals, a_race, &Animal::race);
			return it != animals.end() ? &*it : nullptr;
		}
		std::atomic<bool>         catShown{ true };  // the task that puts the real cat back has run
		constexpr float           kWatchAfter = 3.0f;
		float                     watchAfter{ 0.0f };  // seconds left to check you can walk after the cat
		float                     drivenAfter{ 0.0f };

		// the Activate key: holding it picks a cat up, a tap stays the game's activation
		struct ActivateKey
		{
			bool                down{ false };
			bool                fired{ false };    // held long enough, the pickup was tried
			bool                replaying{ false };
			RE::ObjectRefHandle heldBack;          // a cat activation the game sent on the press, waiting for the release
		} key;

		RE::PlayerCharacter* Player() { return RE::PlayerCharacter::GetSingleton(); }

		RE::Actor* Cat() { return state.cat.get().get(); }

		float Delta() { return std::clamp(RE::GetSecondsSinceLastFrame(), 0.0f, 0.1f); }

		NiPoint3 Flat(const NiPoint3& a) { return { a.x, a.y, 0.0f }; }

		float HeadingTo(const NiPoint3& a_from, const NiPoint3& a_to)
		{
			return std::atan2(a_to.x - a_from.x, a_to.y - a_from.y);
		}

		BodyFrame Frame(RE::Actor* a_player)
		{
			BodyFrame frame;
			auto*     root = a_player->Get3D(false);
			NiPoint3  forward = root ? M::Column(root->world.rotate, 1) : M::Heading(a_player->GetAngleZ()) * NiPoint3{ 0, 1, 0 };
			// the chest's facing: turning and strafing animations twist the upper body against the feet
			if (state.chestKnown) {
				if (const auto chest = Body::ChestRotation(); chest) {
					const NiPoint3 chestForward = *chest * M::Column(state.chestToBody, 1);
					if (M::Length(Flat(chestForward)) > 0.3f) {
						forward = chestForward;
					}
				}
			}
			// facing and lean of the chest (walking and running tip it forward); no sideways roll
			frame.forward = Normalized(forward, { 0.0f, 1.0f, 0.0f });
			frame.right = Normalized(M::Cross(Normalized(Flat(frame.forward), { 0.0f, 1.0f, 0.0f }), NiPoint3{ 0.0f, 0.0f, 1.0f }));
			frame.up = Normalized(M::Cross(frame.right, frame.forward));
			frame.origin = Body::Spine2();
			return frame;
		}

		// ---- the cat's body --------------------------------------------------------------------------------------

		NiPoint3 Radial(const CatPose::Frame& f, float a_angle) { return f.back * std::cos(a_angle) + f.side * std::sin(a_angle); }
		NiPoint3 Tangent(const CatPose::Frame& f, float a_angle) { return f.side * std::cos(a_angle) - f.back * std::sin(a_angle); }

		NiMatrix3 HandIn(const NiPoint3& a_fingers, const NiPoint3& a_palm)
		{
			return M::Align({ 0.0f, 0.0f, 1.0f }, { 0.0f, -1.0f, 0.0f }, a_fingers, a_palm);
		}

		// a hand on the cat's flank facing a_shoulder, fingers sliding under its belly
		Contact FloorContact(bool a_torso, const NiTransform& a_cat, const NiPoint3& a_shoulder)
		{
			const auto&    f = state.frame;
			const NiPoint3 anchor = a_cat * (a_torso ? f.torso : f.pelvis);
			const NiPoint3 toShoulder = a_shoulder - anchor;
			const float    sign = M::Dot(a_cat.rotate * f.side, toShoulder) >= 0.0f ? 1.0f : -1.0f;
			Contact        contact;
			contact.torso = a_torso;
			contact.angle = sign * kFlankAngle;
			contact.radius = Settings::Get().chestRadius;
			contact.along = a_torso ? 1.0f : 0.0f;
			const NiPoint3 radial = Radial(f, contact.angle);
			contact.hand = HandIn(Tangent(f, contact.angle) * sign, radial * -1.0f);
			return contact;
		}

		std::array<Contact, 2> HoldContacts()
		{
			const auto& s = Settings::Get();
			const auto& f = state.frame;
			const float mx = state.mirrored ? -1.0f : 1.0f;

			Contact chest;
			chest.torso = true;
			chest.angle = mx * s.chestAngle * M::kDegrees;
			chest.radius = s.chestRadius;
			chest.along = s.chestAlong;
			const float sign = chest.angle >= 0.0f ? 1.0f : -1.0f;
			chest.hand = HandIn(Tangent(f, chest.angle) * -sign, Radial(f, chest.angle) * -1.0f);

			Contact bottom;
			bottom.torso = false;
			bottom.angle = M::kPi;
			bottom.radius = s.bottomRadius;
			bottom.along = -s.bottomBelow;
			bottom.hand = HandIn(Normalized(f.side * -mx - f.back), f.spine);

			// the chest is held by the hand on the cat's side
			std::array<Contact, 2> contacts;
			contacts[state.mirrored ? Body::kRight : Body::kLeft] = chest;
			contacts[state.mirrored ? Body::kLeft : Body::kRight] = bottom;
			return contacts;
		}

		Contact Blend(const Contact& a, const Contact& b, float t)
		{
			Contact c = b;
			c.angle = M::LerpAngle(a.angle, b.angle, t);
			c.radius = a.radius + (b.radius - a.radius) * t;
			c.along = a.along + (b.along - a.along) * t;
			c.hand = M::Slerp(a.hand, b.hand, t);
			return c;
		}

		// palm point and hand rotation in the world; the contact distances are for a House Cat (kPoseLength chest
		// to hips) and grow / shrink with the cat
		void ContactInWorld(const Contact& a_contact, const NiTransform& a_cat, NiPoint3& a_palm, NiMatrix3& a_rotation)
		{
			const auto&    f = state.frame;
			const float    scale = std::max(a_cat.scale, 0.01f);
			const float    size = std::clamp(2.0f * f.halfLength * scale / kPoseLength, 0.5f, 2.0f);
			const NiPoint3 local = (a_contact.torso ? f.torso : f.pelvis) +
			                       (f.spine * a_contact.along + Radial(f, a_contact.angle) * (a_contact.radius * state.animalGirth) -
			                           f.back * state.animalBelly) * (size / scale);
			a_palm = a_cat * local;
			a_rotation = a_cat.rotate * a_contact.hand;
		}

		// The cat is a bit smaller in your arms (a cat sized for the floor looks too big against your body) and
		// big ones shrink further, so cats of any size (other mods' pets too) fit the holding pose.
		float HeldScale()
		{
			const auto& s = Settings::Get();
			float       scale = state.catScale * std::clamp(s.heldSize, 0.3f, 1.0f);
			const float length = 2.0f * state.frame.halfLength;  // chest to hips, root units
			if (s.maxBodyLength > 0.0f && length > 1e-3f) {
				scale = std::min(scale, s.maxBodyLength / length);
			}
			return std::max(scale * state.animalSize, 0.01f);
		}

		// the cat's root when it's in your arms
		NiTransform HoldTransform(const BodyFrame& a_body)
		{
			const auto&    s = Settings::Get();
			const auto&    f = state.frame;
			const float    mx = state.mirrored ? -1.0f : 1.0f;
			// rounder animals further out, ones with the spine bones along their back (Piglet) closer in
			const float    size = std::clamp(2.0f * f.halfLength * HeldScale() / kPoseLength, 0.5f, 2.0f);
			const float    forward = s.catForward + size * (kPoseDepth * (state.animalGirth - 1.0f) - state.animalBelly);
			const NiPoint3 center = a_body.Point(mx * s.catRight, forward, s.catUp);
			// it shifts its weight a little now and then
			const float    lean = (s.catLean + 3.0f * std::sin(state.clock * 0.31f)) * M::kDegrees;
			const float    tipBack = (s.catBack + 2.0f * std::sin(state.clock * 0.23f + 1.0f)) * M::kDegrees;
			const NiPoint3 spine = Normalized(a_body.Direction(-mx * std::tan(lean), -std::tan(tipBack), 1.0f));
			const float    turn = (s.catTurn + 4.0f * std::sin(state.clock * 0.19f + 2.0f)) * M::kDegrees;
			const NiPoint3 belly = a_body.Direction(-mx * std::sin(turn), std::cos(turn), 0.0f);
			NiTransform    world;
			world.scale = HeldScale();
			world.rotate = M::Align(f.spine, f.back, spine, belly * -1.0f);
			world.translate = center - world.rotate * (f.middle * world.scale);
			return world;
		}

		// from one cat transform to another along a little arc
		NiTransform Between(const NiTransform& a, const NiTransform& b, float t, float a_arc)
		{
			const auto&    f = state.frame;
			NiTransform    world;
			world.scale = a.scale + (b.scale - a.scale) * t;  // shrinks while lifted, grows back while put down
			world.rotate = M::Slerp(a.rotate, b.rotate, t);
			const NiPoint3 middle = M::Lerp(a * f.middle, b * f.middle, t) + NiPoint3{ 0.0f, 0.0f, a_arc * std::sin(M::kPi * t) };
			world.translate = middle - world.rotate * (f.middle * world.scale);
			return world;
		}

		NiPoint3 CatLook(const BodyFrame& a_body, float a_time, float a_petted)
		{
			const auto& s = Settings::Get();
			const float mx = state.mirrored ? -1.0f : 1.0f;
			float       yaw = 20.0f * M::kDegrees;
			float       pitch = -6.0f * M::kDegrees;
			if (s.catLooksAround) {
				// calmer while petted
				const float calm = 1.0f - 0.7f * a_petted;
				yaw += (28.0f * std::sin(a_time * 0.37f) + 9.0f * std::sin(a_time * 1.31f + 1.0f)) * calm * M::kDegrees;
				pitch += (8.0f * std::sin(a_time * 0.53f + 2.0f)) * calm * M::kDegrees;
			}
			// petted: it lifts its chin into your hand
			pitch += 24.0f * a_petted * M::kDegrees;
			const NiPoint3 flat = a_body.forward * std::cos(yaw) - a_body.right * (mx * std::sin(yaw));
			return Normalized(flat * std::cos(pitch) + a_body.up * std::sin(pitch));
		}

		// ---- petting --------------------------------------------------------------------------------------------

		struct Petting
		{
			float weight{ 0.0f };  // 0 holding .. 1 petting
			float stroke{ 0.0f };  // 0 top of the stroke .. 1 bottom
			float lift{ 0.0f };    // hand off the fur on the way back up
		};

		// every few seconds of holding: the hand moves to the cat's chest, strokes it down twice and holds it again
		Petting PettingNow(float a_holdTime)
		{
			const auto& s = Settings::Get();
			Petting     pet;
			if (!s.petCat) {
				return pet;
			}
			const float every = std::max(s.petEvery, 1.0f);
			const float t = std::fmod(a_holdTime, every + kPetTime) - every;
			if (t <= 0.0f) {
				return pet;
			}
			pet.weight = M::Smooth(t / kPetBlend) * M::Smooth((kPetTime - t) / kPetBlend);
			const float strokes = M::Clamp01((t - kPetBlend * 0.6f) / (kPetTime - kPetBlend * 1.2f)) * static_cast<float>(kStrokes);
			const float u = strokes >= static_cast<float>(kStrokes) ? 1.0f : strokes - std::floor(strokes);
			if (u < 0.62f) {
				pet.stroke = M::Smooth(u / 0.62f);  // down along the fur
			} else {
				const float back = (u - 0.62f) / 0.38f;  // up again, the hand a little off
				pet.stroke = 1.0f - M::Smooth(back);
				pet.lift = std::sin(M::kPi * back);
			}
			return pet;
		}

		// the chest hand on the front of the cat's chest, fingers down along the fur
		Contact PetContact(const Petting& a_pet)
		{
			const auto& s = Settings::Get();
			const auto& f = state.frame;
			const float mx = state.mirrored ? -1.0f : 1.0f;
			Contact     contact;
			contact.torso = true;
			contact.angle = mx * 160.0f * M::kDegrees;
			contact.radius = s.chestRadius - 0.5f + 2.0f * a_pet.lift;
			contact.along = s.chestAlong + 4.0f - 10.0f * a_pet.stroke;
			const float    sign = contact.angle >= 0.0f ? 1.0f : -1.0f;
			const NiPoint3 fingers = Normalized(f.spine * -0.8f + Tangent(f, contact.angle) * (-sign * 0.6f));
			contact.hand = HandIn(fingers, Radial(f, contact.angle) * -1.0f);
			return contact;
		}

		// ---- Piglet ---------------------------------------------------------------------------------------------
		// In your arms Piglet wiggles his curly tail and twitches his ears. When you've petted him he turns back to
		// nuzzle your face (you lean in to him), then kicks his legs as if running for joy; halfway to the next
		// petting he has a curious sniff around.

		float Window(float a_t, float a_from, float a_to, float a_ease = 0.35f)
		{
			return M::Smooth((a_t - a_from) / a_ease) * M::Smooth((a_to - a_t) / a_ease);
		}

		void PigletShow(float a_holdTime, float a_petted, CatPose::Mood& a_mood, float& a_nuzzle)
		{
			const auto& s = Settings::Get();
			const float cycle = std::max(s.petEvery, 1.0f) + kPetTime;
			const float c = std::fmod(a_holdTime, cycle);
			const bool  afterPetting = a_holdTime > cycle;  // the first cycle has no petting before it
			const float sniff = Window(c, 3.6f, 5.0f);
			a_nuzzle = afterPetting ? Window(c, 0.0f, 2.7f, 0.5f) : 0.0f;
			const float joy = afterPetting ? Window(c, 2.3f, 3.4f, 0.25f) : 0.0f;
			a_mood.sniff = sniff;
			a_mood.paddle = std::max(joy, 0.35f * a_petted);
			a_mood.tailWag = std::max({ 0.3f, a_petted, a_nuzzle, joy, 0.6f * sniff });
			// ears: perked while sniffing, laid back while petted, a little back while nuzzling
			a_mood.ears = 0.8f * sniff - 0.7f * a_petted - 0.3f * a_nuzzle;
		}

		// ---- controls ------------------------------------------------------------------------------------------

		void DisableControls(UEFlag a_flags)
		{
			auto*        map = RE::ControlMap::GetSingleton();
			const UEFlag fresh = static_cast<UEFlag>(std::to_underlying(a_flags) & ~std::to_underlying(state.disabled));
			if (map && fresh != UEFlag::kNone) {
				map->ToggleControls(fresh, false, false);
				state.disabled = static_cast<UEFlag>(std::to_underlying(state.disabled) | std::to_underlying(fresh));
			}
		}

		void EnableControls(UEFlag a_flags)
		{
			auto*        map = RE::ControlMap::GetSingleton();
			const UEFlag ours = static_cast<UEFlag>(std::to_underlying(a_flags) & std::to_underlying(state.disabled));
			if (map && ours != UEFlag::kNone) {
				map->ToggleControls(ours, true, false);
				state.disabled = static_cast<UEFlag>(std::to_underlying(state.disabled) & ~std::to_underlying(ours));
			}
		}

		// sprint, sneak and POV keys off (turning sneaking off through the control map would make you stand up; the
		// POV switch control also stops the mouse wheel zoom)
		void BlockKeys(bool a_block)
		{
			auto* controls = RE::PlayerControls::GetSingleton();
			if (!controls || state.keysBlocked == a_block) {
				return;
			}
			if (controls->sprintHandler) {
				controls->sprintHandler->SetInputEventHandlingEnabled(!a_block);
			}
			if (controls->sneakHandler) {
				controls->sneakHandler->SetInputEventHandlingEnabled(!a_block);
			}
			if (controls->togglePOVHandler) {
				controls->togglePOVHandler->SetInputEventHandlingEnabled(!a_block);
			}
			state.keysBlocked = a_block;
		}

		constexpr auto kAlwaysOff = static_cast<UEFlag>(std::to_underlying(UEFlag::kFighting) | std::to_underlying(UEFlag::kJumping) |
		                                                std::to_underlying(UEFlag::kActivate));

		// You can zoom while carrying the cat, but not into 1st person: the zoom stops just short of where the game
		// switches, and if a fast scroll gets there anyway, back to 3rd person.
		void KeepThirdPerson()
		{
			auto* camera = RE::PlayerCamera::GetSingleton();
			if (!camera) {
				return;
			}
			if (camera->IsInFirstPerson()) {
				SKSE::GetTaskInterface()->AddTask([]() {
					if (auto* camera = RE::PlayerCamera::GetSingleton(); camera && camera->IsInFirstPerson() && Carry::Active()) {
						camera->ForceThirdPerson();
					}
				});
				return;
			}
			if (!camera->IsInThirdPerson() || !camera->currentState) {
				return;
			}
			static const float minZoom = []() {
				auto* ini = RE::INISettingCollection::GetSingleton();
				auto* setting = ini ? ini->GetSetting("fMinCurrentZoom:Camera") : nullptr;
				return setting ? setting->GetFloat() : -0.2f;
			}();
			auto* third = static_cast<RE::ThirdPersonState*>(camera->currentState.get());
			if (third->targetZoomOffset < minZoom + 0.05f) {
				third->targetZoomOffset = minZoom + 0.05f;
			}
		}
		constexpr auto kStill = static_cast<UEFlag>(std::to_underlying(UEFlag::kMovement) | std::to_underlying(UEFlag::kLooking));
		constexpr auto kOurControls = static_cast<UEFlag>(std::to_underlying(kAlwaysOff) | std::to_underlying(kStill));

		// Picking up and putting down: no walking, no looking (turning the camera turned you away from the cat),
		// and you're kept on your spot facing where you face now. Called again to take a new spot.
		void Lock(RE::PlayerCharacter* a_player)
		{
			DisableControls(kStill);
			state.locked = true;
			state.lockHeading = a_player->GetAngleZ();
			state.lockPos = a_player->GetPosition();
		}

		void Unlock()
		{
			state.locked = false;
			EnableControls(kStill);
		}

		// Movement and looking stay on while you hold the cat and right after it. Now and then they were still off a
		// second into the hold (1.5.2 log: controls FFFFFB98 = exactly our pickup lock, right after the kneel had left
		// you "sitting"): you couldn't walk or turn until something switched them on again.
		int controlsFixed = 0;

		void KeepControlsOn(const char* a_when)
		{
			auto* map = RE::ControlMap::GetSingleton();
			auto* ui = RE::UI::GetSingleton();
			if (!map || map->AreControlsEnabled(kStill) ||
				(ui && (ui->GameIsPaused() || ui->IsMenuOpen(RE::Console::MENU_NAME) || ui->IsMenuOpen(RE::DialogueMenu::MENU_NAME)))) {
				return;
			}
			if (++controlsFixed <= 10) {
				logs::info("{}: movement {} / looking {} were off (controls {:08X}), switching them on", a_when, map->IsMovementControlsEnabled(),
					map->IsLookingControlsEnabled(), map->GetRuntimeData().enabledControls.underlying());
			}
			map->ToggleControls(kStill, true, false);
			state.disabled = static_cast<UEFlag>(std::to_underlying(state.disabled) & ~std::to_underlying(kStill));
		}

		// anything else that turned or moved you (other mods, the kneel clip) is undone
		void KeepStill(RE::PlayerCharacter* a_player)
		{
			if (!state.locked || state.phase == Phase::kReach) {  // the reach turns and slides you itself
				return;
			}
			const float turned = std::remainder(a_player->GetAngleZ() - state.lockHeading, 2.0f * M::kPi);
			if (std::abs(turned) > 0.002f) {
				a_player->SetHeading(state.lockHeading);
			}
			const NiPoint3 pos = a_player->GetPosition();
			if (M::Length(Flat(pos - state.lockPos)) > 0.5f) {
				a_player->SetPosition({ state.lockPos.x, state.lockPos.y, pos.z }, true);
			}
		}

		// ---- the real cat --------------------------------------------------------------------------------------

		void FreezeCat(RE::Actor* a_cat)
		{
			state.catAI = a_cat->IsAIEnabled();
			if (state.catAI) {
				a_cat->EnableAI(false);
			}
		}

		// The real cat's own 3D is what you hold (copies of its Oldrim format mesh render with stretched paws). Its
		// actor (its capsule) is parked on the ground behind you, frozen; its 3D is placed in your arms every frame.
		// (Parked under the floor, the game pushed the capsule back up into yours and you slid around.)
		void ParkCat()
		{
			SKSE::GetTaskInterface()->AddTask([handle = state.cat]() {
				auto  cat = handle.get();
				auto* player = RE::PlayerCharacter::GetSingleton();
				if (!cat || !player) {
					return;
				}
				if (cat->GetParentCell() != player->GetParentCell()) {
					cat->MoveTo(player);
				}
				const NiPoint3 back = M::Heading(player->GetAngleZ()) * NiPoint3{ 0.0f, -kParkBehind, 0.0f };
				cat->SetPosition(player->GetPosition() + back, true);
			});
		}

		void TakeCat()
		{
			if (auto* cat = Cat(); cat && !state.catHidden) {
				state.catHidden = true;
				state.catRoot = cat->Get3D();
				state.followTimer = kFollowEvery;
				ParkCat();
			}
		}

		void KeepCat(RE::PlayerCharacter* a_player, float a_delta)
		{
			auto* cat = Cat();
			if (!cat || !state.catHidden) {
				return;
			}
			auto* root = cat->Get3D();
			if (root != state.catRoot) {
				// its 3D was unloaded (cell change) or loaded again: hold the new one
				state.catRoot = root;
				if (root) {
					logs::info("The cat's 3D was loaded again; holding the new one");
					CatPose::Hold(cat);
				} else {
					CatPose::Release();
				}
			}
			state.followTimer -= a_delta;
			if (state.followTimer > 0.0f) {
				return;
			}
			state.followTimer = kFollowEvery;
			// moved only when needed: moving the cat puts its 3D on its own spot for a moment
			auto*      catCell = cat->GetParentCell();
			auto*      playerCell = a_player->GetParentCell();
			const bool otherInterior = catCell != playerCell && ((catCell && catCell->IsInteriorCell()) || (playerCell && playerCell->IsInteriorCell()));
			if (!root || otherInterior || M::Length(Flat(cat->GetPosition() - a_player->GetPosition())) > kParkAgain) {
				ParkCat();
			}
		}

		// the real cat back into the world at a root transform
		void PutCatBack(const NiTransform& a_root, float a_heading)
		{
			auto* cat = Cat();
			if (!cat) {
				return;
			}
			const bool ai = state.catAI;
			CatPose::Release();
			state.catHidden = false;
			state.catBack = true;
			catShown = false;
			SKSE::GetTaskInterface()->AddTask([handle = state.cat, pos = a_root.translate, a_heading, ai]() {
				auto  cat = handle.get();
				auto* player = RE::PlayerCharacter::GetSingleton();
				if (!cat || !player) {
					catShown = true;
					return;
				}
				if (cat->GetParentCell() != player->GetParentCell()) {
					cat->MoveTo(player);
				}
				if (cat->IsDisabled()) {
					cat->Enable(false);
				}
				cat->SetPosition(pos, true);
				cat->SetAngle({ 0.0f, 0.0f, a_heading });
				if (auto* root = cat->Get3D(); root) {
					root->SetAppCulled(false);
				}
				if (ai) {
					cat->EnableAI(true);
				}
				catShown = true;
			});
		}

		// ---- phases --------------------------------------------------------------------------------------------

		// The vanilla kneel runs through the furniture states of the behavior graph and leaves you "sitting"
		// (sit state 3): you couldn't move after putting the cat down. Cleared once the stand-up clip is done.
		void StandUpFully(RE::PlayerCharacter* a_player)
		{
			auto* actorState = a_player ? a_player->AsActorState() : nullptr;
			if (!actorState || actorState->actorState1.sitSleepState == RE::SIT_SLEEP_STATE::kNormal) {
				return;
			}
			logs::info("Still in sit state {} after the kneel: standing up", std::to_underlying(actorState->actorState1.sitSleepState));
			actorState->actorState1.sitSleepState = RE::SIT_SLEEP_STATE::kNormal;
		}

		// You couldn't walk after some carries: the kneel goes through the behavior graph's furniture states, which
		// can leave you "sitting" or driven by the animation. While you hold the cat and for a few seconds after
		// it, both are cleared when you aren't really on a piece of furniture.
		void Unstick(RE::PlayerCharacter* a_player, float a_delta, float& a_drivenFor, const char* a_when)
		{
			auto* actorState = a_player->AsActorState();
			if (!actorState || a_player->GetOccupiedFurniture() || a_player->IsOnMount()) {
				a_drivenFor = 0.0f;
				return;
			}
			if (actorState->actorState1.sitSleepState != RE::SIT_SLEEP_STATE::kNormal) {
				logs::info("{}: sit state {} without furniture, standing up", a_when, std::to_underlying(actorState->actorState1.sitSleepState));
				actorState->actorState1.sitSleepState = RE::SIT_SLEEP_STATE::kNormal;
			}
			bool driven = false;
			if (a_player->GetGraphVariableBool("bAnimationDriven", driven) && driven) {
				a_drivenFor += a_delta;
				if (a_drivenFor > 0.75f) {
					logs::info("{}: the animation still drove you after {:.1f} s, back to the default state", a_when, a_drivenFor);
					a_player->NotifyAnimationGraph(kDefaultState);
					a_player->SetGraphVariableBool("bAnimationDriven", false);
					a_drivenFor = -1e9f;  // once until it goes off by itself: no fight with a mod that keeps it on
				}
			} else {
				a_drivenFor = 0.0f;
			}
		}

		void Finish()
		{
			if (state.catHidden) {
				auto*       player = Player();
				NiTransform spot;
				spot.translate = player ? player->GetPosition() + M::Heading(player->GetAngleZ()) * NiPoint3{ 0.0f, 30.0f, 0.0f } : NiPoint3{};
				PutCatBack(spot, player ? player->GetAngleZ() : 0.0f);
			}
			EnableControls(static_cast<UEFlag>(0xFFFFFFFF));
			BlockKeys(false);
			// every control we may have switched off is on again, even if our bookkeeping missed one
			if (auto* map = RE::ControlMap::GetSingleton(); map) {
				if (!map->AreControlsEnabled(kOurControls)) {
					logs::info("Some controls were still off after the cat (movement {}, looking {}), switching them on", map->IsMovementControlsEnabled(),
						map->IsLookingControlsEnabled());
					map->ToggleControls(kOurControls, true, false);
				}
			}
			if (auto* player = Player(); player) {
				bool driven = false;
				player->GetGraphVariableBool("bAnimationDriven", driven);
				logs::info("Done with the cat: sitting state {}, animation driven {}, movement {}", std::to_underlying(player->AsActorState()->actorState1.sitSleepState),
					driven, RE::ControlMap::GetSingleton() && RE::ControlMap::GetSingleton()->IsMovementControlsEnabled());
			}
			watchAfter = kWatchAfter;
			drivenAfter = 0.0f;
			Body::Release();
			CatPose::Release();
			if (state.wasFirstPerson) {
				if (auto* camera = RE::PlayerCamera::GetSingleton(); camera) {
					camera->ForceFirstPerson();
				}
			}
			state = {};
		}

		// stop right now (combat, swimming, ...): the cat lands where it is
		void Drop(const char* a_why)
		{
			logs::info("Letting go of the cat: {}", a_why);
			auto* player = Player();
			if (CatPose::Active()) {
				NiTransform root = CatPose::Root()->world;
				const float heading = std::atan2(M::Column(root.rotate, 1).x, M::Column(root.rotate, 1).y);
				root.translate.z = player ? player->GetPositionZ() : root.translate.z;
				PutCatBack(root, heading);
			} else if (auto* cat = Cat(); cat && state.catAI) {
				cat->EnableAI(true);
			}
			if (state.kneeling && player) {
				player->NotifyAnimationGraph(kDefaultState);
				StandUpFully(player);
			}
			Finish();
		}

		bool PlayerBusy(RE::PlayerCharacter* a_player, bool a_starting)
		{
			auto* actorState = a_player->AsActorState();
			if (a_player->IsDead() || a_player->IsInCombat() || a_player->IsOnMount() || a_player->IsInRagdollState() ||
				actorState->IsSwimming() || actorState->IsBleedingOut() ||
				actorState->GetKnockState() != RE::KNOCK_STATE_ENUM::kNormal) {
				return true;
			}
			// our own kneel goes through the furniture states of the behavior graph: sitting only counts outside it
			const bool ownKneel = !a_starting && state.kneeling && state.phase != Phase::kHold;
			if (!ownKneel && actorState->actorState1.sitSleepState != RE::SIT_SLEEP_STATE::kNormal) {
				return true;
			}
			if (a_starting) {
				if (auto* ui = RE::UI::GetSingleton(); ui && ui->GameIsPaused()) {
					return true;
				}
			}
			return false;
		}

		void StartReach()
		{
			auto* player = Player();
			auto* cat = Cat();
			if (!player || !cat || !cat->Get3D() || !Body::Setup(player)) {
				logs::warn("Couldn't start the pickup (no 3D or no arms)");
				if (cat && state.catAI) {
					cat->EnableAI(true);
				}
				Finish();
				return;
			}
			const auto& s = Settings::Get();
			auto*       root = cat->Get3D();
			state.ground = root->world;
			state.catScale = root->world.scale;
			state.frame = CatPose::Measure(root);
			Body::BeginFrame();

			// which hand holds the chest: the side its head points to (or the setting)
			state.chestKnown = false;
			const BodyFrame body = Frame(player);
			if (const auto chest = Body::ChestRotation(); chest) {
				state.chestToBody = chest->Transpose() * M::FromColumns(body.right, body.forward, body.up);
				state.chestKnown = true;
			}
			const NiPoint3  head = state.ground.rotate * state.frame.spine;
			state.mirrored = s.side == 2 || (s.side == 0 && M::Dot(head, body.right) > 0.0f);
			const int chest = state.mirrored ? Body::kRight : Body::kLeft;
			for (int hand = 0; hand < 2; ++hand) {
				state.floorContact[hand] = FloorContact(hand == chest, state.ground, Body::Shoulder(static_cast<Body::Side>(hand)));
			}

			// face the cat and step to a comfortable distance
			const NiPoint3 catPos = state.ground * state.frame.middle;
			const NiPoint3 playerPos = player->GetPosition();
			const float    height = catPos.z - playerPos.z;
			state.kneeling = s.kneel && height <= s.kneelBelow;
			state.startPos = playerPos;
			state.startHeading = player->GetAngleZ();
			state.targetHeading = HeadingTo(playerPos, catPos);
			const NiPoint3 away = Normalized(Flat(playerPos - catPos), body.forward * -1.0f);
			const float    distance = M::Length(Flat(playerPos - catPos));
			const float    want = state.kneeling ? kReachDistance : kReachDistance - 4.0f;
			const float    target = std::clamp(distance, kMinDistance, want);
			const float    slide = std::clamp(distance - target, -kMaxSlide, kMaxSlide);
			state.standPos = playerPos - away * slide;
			state.standPos.z = playerPos.z;

			if (state.kneeling && !player->NotifyAnimationGraph(kKneelEnter)) {
				logs::info("The kneel animation didn't start, reaching down standing");
				state.kneeling = false;
			}
			logs::info("Picking up {}: {:.0f} away, body {:.0f} above your feet, {}, held on your {}", cat->GetDisplayFullName(), distance,
				height, state.kneeling ? "kneeling" : "standing", state.mirrored ? "right" : "left");
			logs::info("Its size: scale {:.2f}, {:.1f} long chest to hips; in your arms scale {:.2f}, {:.1f} long", state.catScale,
				2.0f * state.frame.halfLength * state.catScale, HeldScale(), 2.0f * state.frame.halfLength * HeldScale());
			state.phase = Phase::kReach;
			state.time = 0.0f;
		}

		void StartGrab()
		{
			auto* player = Player();
			auto* cat = Cat();
			auto* parent = player ? player->Get3D(false) : nullptr;
			if (!cat || !parent || !CatPose::Hold(cat)) {
				logs::warn("Couldn't take the cat's 3D");
				Drop("no 3D");
				return;
			}
			state.ground = CatPose::Root()->world;
			TakeCat();
			Lock(player);  // where the reach left you
			state.phase = Phase::kGrab;
			state.time = 0.0f;
		}

		void StartLift()
		{
			if (state.kneeling) {
				auto* player = Player();
				if (!player->NotifyAnimationGraph(kKneelExit)) {
					player->NotifyAnimationGraph(kDefaultState);
				}
			}
			state.holdContact = HoldContacts();
			state.phase = Phase::kLift;
			state.time = 0.0f;
		}

		void StartHold()
		{
			StandUpFully(Player());
			Unlock();
			state.drivenFor = 0.0f;
			state.phase = Phase::kHold;
			state.time = 0.0f;
		}

		void StartLower()
		{
			auto* player = Player();
			const auto& s = Settings::Get();
			Lock(player);
			const BodyFrame body = Frame(player);
			const float     mx = state.mirrored ? -1.0f : 1.0f;

			// lands side on in front of you, head to the side it was held on
			const NiPoint3 head = body.right * -mx;
			state.dropHeading = std::atan2(head.x, head.y);
			NiTransform drop;
			drop.scale = state.catScale;
			drop.rotate = M::Heading(state.dropHeading);
			const NiPoint3 spot = player->GetPosition() + body.forward * kReachDistance;
			drop.translate = spot - drop.rotate * (Flat(state.frame.middle) * drop.scale);
			drop.translate.z = player->GetPositionZ();
			state.ground = drop;
			const int chest = state.mirrored ? Body::kRight : Body::kLeft;
			for (int hand = 0; hand < 2; ++hand) {
				state.floorContact[hand] = FloorContact(hand == chest, drop, Body::Shoulder(static_cast<Body::Side>(hand)));
			}
			state.kneeling = s.kneel && player->NotifyAnimationGraph(kKneelEnter);
			state.phase = Phase::kLower;
			logs::info("Putting the cat down (kneeling: {})", state.kneeling);
			state.time = 0.0f;
		}

		// attacked: the cat goes down beside you right away and your hands are free
		void StartQuickDrop()
		{
			auto* player = Player();
			logs::info("Attacked while holding the cat: putting it down fast");
			Unlock();
			if (state.kneeling) {
				player->NotifyAnimationGraph(kDefaultState);
				StandUpFully(player);
				state.kneeling = false;
			}
			const BodyFrame body = Frame(player);
			const float     mx = state.mirrored ? -1.0f : 1.0f;
			state.dropFrom = CatPose::Root()->world;
			state.dropHeading = std::atan2(body.forward.x, body.forward.y);
			NiTransform drop;
			drop.scale = state.catScale;
			drop.rotate = M::Heading(state.dropHeading);
			const NiPoint3 spot = player->GetPosition() + body.forward * 26.0f - body.right * (mx * 14.0f);
			drop.translate = spot - drop.rotate * (Flat(state.frame.middle) * drop.scale);
			drop.translate.z = player->GetPositionZ();
			state.ground = drop;
			if (state.phase == Phase::kGrab) {
				state.holdContact = state.floorContact;
			}
			state.putDownRequest = false;
			state.phase = Phase::kQuickDrop;
			logs::info("Quick drop (kneeling: {})", state.kneeling);
			state.time = 0.0f;
		}

		void StartLetGo()
		{
			PutCatBack(state.ground, state.dropHeading);
			state.phase = Phase::kLetGo;
			logs::info("The cat is back on the floor (kneeling: {})", state.kneeling);
			state.time = 0.0f;
		}

		void StartStand()
		{
			if (state.kneeling) {
				auto* player = Player();
				if (!player->NotifyAnimationGraph(kKneelExit)) {
					player->NotifyAnimationGraph(kDefaultState);
				}
			}
			Body::Release();
			state.phase = Phase::kStand;
			logs::info("Standing up (kneeling: {})", state.kneeling);
			state.time = 0.0f;
		}

		// ---- per frame -----------------------------------------------------------------------------------------

		struct Pose
		{
			NiTransform              cat;
			bool                     placed{ false };  // we place and pose the cat's 3D (else the game does)
			std::array<Contact, 2>   contacts;
			std::array<NiPoint3, 2>  poles;
			float                    weight{ 0.0f };
			float                    curl{ 0.0f };
			float                    catHold{ 0.0f };
			float                    leanAmount{ 0.0f };  // 0..1 of the lean the reach needs
			float                    look{ 0.0f };
			float                    petted{ 0.0f };      // the chest hand strokes the cat
			CatPose::Mood            mood;                // Piglet's own show
			float                    nuzzle{ 0.0f };      // Piglet turns to your face, you lean in
		};

		std::array<NiPoint3, 2> ReachPoles(const BodyFrame& a_body)
		{
			return { a_body.Direction(-1.0f, 0.15f, -0.7f), a_body.Direction(1.0f, 0.15f, -0.7f) };
		}

		std::array<NiPoint3, 2> HoldPoles(const BodyFrame& a_body)
		{
			const int   chest = state.mirrored ? Body::kRight : Body::kLeft;
			std::array<NiPoint3, 2> poles;
			for (int hand = 0; hand < 2; ++hand) {
				const float out = hand == Body::kLeft ? -1.0f : 1.0f;
				poles[hand] = hand == chest ? a_body.Direction(out * 1.0f, 0.5f, -0.6f) : a_body.Direction(out * 1.0f, -0.25f, -0.8f);
			}
			return poles;
		}

		std::array<NiPoint3, 2> BlendPoles(const std::array<NiPoint3, 2>& a, const std::array<NiPoint3, 2>& b, float t)
		{
			return { Normalized(M::Lerp(a[0], b[0], t)), Normalized(M::Lerp(a[1], b[1], t)) };
		}

		std::array<Contact, 2> BlendContacts(const std::array<Contact, 2>& a, const std::array<Contact, 2>& b, float t)
		{
			return { Blend(a[0], b[0], t), Blend(a[1], b[1], t) };
		}

		void Step(RE::PlayerCharacter* a_player, float a_delta)
		{
			state.time += a_delta;
			state.clock += a_delta;
			const float t = state.time;

			switch (state.phase) {
			case Phase::kSheathe:
				if (a_player->AsActorState()->GetWeaponState() == RE::WEAPON_STATE::kSheathed) {
					StartReach();
				} else if (t > kSheatheTimeout) {
					logs::info("The weapon didn't go away, not picking the cat up");
					Drop("weapon still out");
				}
				return;
			case Phase::kReach:
				{
					const float turn = M::Smooth(t / kTurnTime);
					a_player->SetHeading(M::LerpAngle(state.startHeading, state.targetHeading, turn));
					const float slide = M::Smooth(t / kSlideTime);
					NiPoint3    pos = M::Lerp(state.startPos, state.standPos, slide);
					pos.z = a_player->GetPositionZ();
					if (M::Length(Flat(pos - a_player->GetPosition())) > 0.01f) {
						a_player->SetPosition(pos, true);
					}
					if (t >= (state.kneeling ? kKneelReachTime : kStandReachTime)) {
						StartGrab();
					}
				}
				return;
			case Phase::kGrab:
				if (t >= kGrabTime) {
					StartLift();
				}
				return;
			case Phase::kLift:
				if (t >= kLiftTime) {
					StartHold();
				}
				return;
			case Phase::kHold:
				if (state.putDownRequest) {
					state.putDownRequest = false;
					StartLower();
				}
				return;
			case Phase::kLower:
				if (t >= kLowerTime) {
					StartLetGo();
				}
				return;
			case Phase::kLetGo:
				{
					auto* cat = Cat();
					const bool back = cat && cat->Get3D() && catShown && !cat->IsDisabled();
					if (CatPose::Active() && (back || t > kAppearTimeout)) {
						CatPose::Release();
						state.time = 0.0f;
					} else if (!CatPose::Active() && t >= kLetGoTime) {
						StartStand();
					}
				}
				return;
			case Phase::kStand:
				if (t >= kStandTime) {
					StandUpFully(a_player);
					Finish();
				}
				return;
			case Phase::kQuickDrop:
				if (!state.catBack && t >= kQuickLand) {
					PutCatBack(state.ground, state.dropHeading);
				}
				if (t >= kQuickTime) {
					Finish();
				}
				return;
			default:
				return;
			}
		}

		std::optional<Pose> PoseNow(const BodyFrame& a_body)
		{
			Pose        pose;
			const float t = state.time;
			const auto& s = Settings::Get();
			switch (state.phase) {
			case Phase::kReach:
				{
					auto* cat = Cat();
					auto* root = cat ? cat->Get3D() : nullptr;
					if (!root) {
						return std::nullopt;
					}
					pose.cat = root->world;
					const float start = state.kneeling ? kKneelReachStart : 0.0f;
					const float length = (state.kneeling ? kKneelReachTime : kStandReachTime) - start;
					pose.weight = M::Smooth((t - start) / length);
					pose.contacts = state.floorContact;
					pose.poles = ReachPoles(a_body);
					pose.leanAmount = pose.weight;
					pose.look = M::Smooth(t / 0.4f);
				}
				break;
			case Phase::kGrab:
				pose.cat = state.ground;
				pose.placed = true;
				pose.weight = 1.0f;
				pose.curl = M::Smooth(t / kGrabTime);
				pose.contacts = state.floorContact;
				pose.poles = ReachPoles(a_body);
				pose.leanAmount = 1.0f;
				pose.look = 1.0f;
				break;
			case Phase::kLift:
				{
					const float e = M::Smooth(t / kLiftTime);
					pose.cat = Between(state.ground, HoldTransform(a_body), e, 4.0f);
					pose.placed = true;
					pose.weight = 1.0f;
					pose.curl = 1.0f;
					pose.contacts = BlendContacts(state.floorContact, state.holdContact, M::Smooth((t - 0.1f) / (kLiftTime - 0.3f)));
					pose.poles = BlendPoles(ReachPoles(a_body), HoldPoles(a_body), e);
					pose.catHold = e;
					pose.leanAmount = 1.0f - e;
					pose.look = 1.0f - e;
				}
				break;
			case Phase::kHold:
				{
					pose.cat = HoldTransform(a_body);
					pose.placed = true;
					pose.weight = 1.0f;
					pose.curl = 1.0f;
					pose.contacts = state.holdContact;
					pose.poles = HoldPoles(a_body);
					pose.catHold = 1.0f;
					// a glance at the cat now and then
					const float cycle = std::fmod(state.clock, 9.0f);
					pose.look = cycle > 6.5f ? 0.6f * std::sin(M::kPi * (cycle - 6.5f) / 2.5f) : 0.0f;
					// and now and then a stroke over its chest (and a look at it meanwhile)
					const Petting pet = PettingNow(t);
					if (pet.weight > 0.0f) {
						const int chest = state.mirrored ? Body::kRight : Body::kLeft;
						pose.contacts[chest] = Blend(state.holdContact[chest], PetContact(pet), pet.weight);
						pose.petted = pet.weight;
						pose.look = std::max(pose.look, 0.7f * pet.weight);
					}
					if (state.piglet) {
						PigletShow(t, pose.petted, pose.mood, pose.nuzzle);
						pose.look = std::max(pose.look, 0.6f * pose.nuzzle);
					}
				}
				break;
			case Phase::kLower:
				{
					const float e = M::Smooth((t - kLowerStart) / (kLowerTime - kLowerStart));
					pose.cat = Between(HoldTransform(a_body), state.ground, e, 3.0f);
					pose.placed = true;
					pose.weight = 1.0f;
					pose.curl = 1.0f;
					pose.contacts = BlendContacts(state.holdContact, state.floorContact, M::Smooth((t - kLowerStart) / (kLowerTime - kLowerStart - 0.15f)));
					pose.poles = BlendPoles(HoldPoles(a_body), ReachPoles(a_body), e);
					pose.catHold = 1.0f - e;
					pose.leanAmount = e;
					pose.look = M::Smooth(t / 0.5f);
				}
				break;
			case Phase::kQuickDrop:
				{
					const float e = M::Smooth(t / kQuickLand);
					pose.cat = Between(state.dropFrom, state.ground, e, 2.0f);
					pose.placed = true;
					pose.weight = 1.0f - M::Smooth((t - 0.15f) / (kQuickTime - 0.15f));
					pose.curl = pose.weight;
					pose.contacts = state.holdContact;
					pose.poles = HoldPoles(a_body);
					pose.catHold = 1.0f - e;
				}
				break;
			case Phase::kLetGo:
				pose.cat = state.ground;
				pose.placed = CatPose::Active();
				pose.contacts = state.floorContact;
				pose.poles = ReachPoles(a_body);
				// hands stay on the cat until its actor has its 3D back, then leave it
				pose.weight = CatPose::Active() ? 1.0f : 1.0f - M::Smooth(t / kLetGoTime);
				pose.curl = pose.weight;
				pose.leanAmount = pose.weight;
				pose.look = pose.weight;
				break;
			default:
				return std::nullopt;
			}
			if (!s.lookAtCat) {
				pose.look = 0.0f;
			}
			return pose;
		}

		void ApplyPose(RE::PlayerCharacter* a_player, float a_delta)
		{
			const BodyFrame body = Frame(a_player);
			auto            pose = PoseNow(body);
			if (!pose) {
				return;
			}
			const auto& s = Settings::Get();

			if (pose->placed && CatPose::Active()) {
				NiPoint3 look = CatLook(body, state.clock, pose->petted);
				if (pose->nuzzle > 0.0f) {
					const NiPoint3 toYou = Normalized(Body::Head() - pose->cat * state.frame.torso, look);
					const NiPoint3 rub = body.right * (0.25f * std::sin(state.clock * 5.0f));
					look = Normalized(M::Lerp(look, Normalized(toYou + rub), 0.85f * pose->nuzzle), look);
				}
				// long heads (Piglet's snout) stay mostly in line with the body: only a little of the look
				const Head head = CurrentHead();
				if (head.turn < 1.0f) {
					const NiPoint3 along = Normalized(pose->cat.rotate * state.frame.spine, look);
					look = Normalized(M::Lerp(along, look, std::max(head.turn, 0.0f)), look);
				}
				CatPose::Mood mood = pose->mood;
				const float   mx = state.mirrored ? -1.0f : 1.0f;
				mood.headPitch = head.pitch * M::kDegrees;
				mood.headYaw = mx * head.yaw * M::kDegrees;
				mood.headRoll = mx * head.roll * M::kDegrees;
				mood.neck = head.neck;
				CatPose::Place(pose->cat, pose->catHold, state.clock, look, s.tailSway ? 1.0f : 0.0f, s.pawsForward, s.legHang,
					mood);
			}

			Body::Goals goals;
			goals.right = body.right;
			for (int hand = 0; hand < 2; ++hand) {
				auto& goal = goals.hands[hand];
				ContactInWorld(pose->contacts[hand], pose->cat, goal.palm, goal.rotation);
				goal.pole = pose->poles[hand];
				goal.weight = pose->weight;
				goal.curl = s.fingerCurl * pose->curl;
			}
			// an open hand for petting
			if (pose->petted > 0.0f) {
				auto& chest = goals.hands[state.mirrored ? Body::kRight : Body::kLeft];
				chest.curl *= 1.0f - 0.75f * pose->petted;
			}
			// bend forward as far as the reach needs (plus a little), eased
			float leanTarget = 0.0f;
			if (pose->leanAmount > 0.0f) {
				Body::Goals full = goals;
				for (auto& goal : full.hands) {
					goal.weight = 1.0f;
				}
				leanTarget = (Body::LeanFor(full, s.maxLean * M::kDegrees) + kBaseLean) * pose->leanAmount;
			}
			state.lean = M::Follow(state.lean, leanTarget, kLeanRate, a_delta);
			goals.lean = state.lean;
			// your head away from the cat's head: tilted and turned to the other side, a little back
			if (pose->catHold > 0.0f && s.headAway > 0.0f) {
				const float mx = state.mirrored ? -1.0f : 1.0f;
				const float away = s.headAway * M::kDegrees * pose->catHold * (1.0f - 0.8f * pose->nuzzle);
				goals.headTurn = M::AxisAngle(body.forward, mx * 0.7f * away) * M::AxisAngle(body.up, -mx * away) *
				                 M::AxisAngle(body.right, 0.3f * away);
			}
			state.look = M::Follow(state.look, pose->look, 5.0f, a_delta);
			if (state.look > 0.01f) {
				goals.lookAt = pose->cat * state.frame.torso;
				goals.lookWeight = state.look;
			}
			Body::Apply(goals);
		}

		// in combat, or your health just went down
		bool UnderAttack(RE::PlayerCharacter* a_player)
		{
			auto*       values = a_player->AsActorValueOwner();
			const float health = values ? values->GetActorValue(RE::ActorValue::kHealth) : 0.0f;
			const bool  hit = state.health >= 0.0f && health < state.health - 0.5f;
			state.health = health;
			return hit || a_player->IsInCombat();
		}

		bool StillFine(RE::PlayerCharacter* a_player)
		{
			if (state.phase == Phase::kQuickDrop) {
				if (!a_player->Get3D(false)) {
					Drop("your 3D is gone");
					return false;
				}
				return true;
			}
			if (UnderAttack(a_player)) {
				if (CatPose::Active() && state.phase != Phase::kLetGo) {
					StartQuickDrop();
					return true;
				}
				Drop("you're under attack");
				return false;
			}
			if (PlayerBusy(a_player, false)) {
				Drop("you're busy (combat, swimming, knocked down...)");
				return false;
			}
			if (!a_player->Get3D(false)) {
				Drop("your 3D is gone");
				return false;
			}
			if (state.phase == Phase::kReach && (!Cat() || !Cat()->Get3D())) {
				Drop("the cat's 3D is gone");
				return false;
			}
			return true;
		}
		// a tap on Activate with the cat in your arms still opens doors
		void ActivateDoor()
		{
			if (auto* ui = RE::UI::GetSingleton(); ui && (ui->GameIsPaused() || ui->IsMenuOpen(RE::Console::MENU_NAME))) {
				return;
			}
			auto* pick = RE::CrosshairPickData::GetSingleton();
			auto  target = pick ? pick->GetActiveTarget().get() : nullptr;
			if (!target || !target->GetBaseObject() || !target->GetBaseObject()->Is(RE::FormType::Door)) {
				return;
			}
			SKSE::GetTaskInterface()->AddTask([handle = target->GetHandle()]() {
				if (auto door = handle.get(); door) {
					door->ActivateRef(RE::PlayerCharacter::GetSingleton(), 0, nullptr, 1, false);
				}
			});
		}
	}

	namespace
	{
		std::string Trim(const std::string& a_text)
		{
			const auto first = a_text.find_first_not_of(" \t\r\n");
			const auto last = a_text.find_last_not_of(" \t\r\n");
			return first == std::string::npos ? std::string{} : a_text.substr(first, last - first + 1);
		}

		void AddRaces(const std::string& a_list, const Animal& a_animal)
		{
			auto*       data = RE::TESDataHandler::GetSingleton();
			std::size_t start = 0;
			while (start <= a_list.size()) {
				const auto  end = std::min(a_list.find(',', start), a_list.size());
				std::string entry = a_list.substr(start, end - start);
				start = end + 1;
				const auto bar = entry.find('|');
				if (bar == std::string::npos) {
					continue;
				}
				const std::string plugin = Trim(entry.substr(0, bar));
				const std::string id = Trim(entry.substr(bar + 1));
				RE::FormID        local = 0;
				try {
					local = static_cast<RE::FormID>(std::stoul(id, nullptr, 16));
				} catch (...) {
					logs::warn("Bad race entry '{}'", entry);
					continue;
				}
				auto* race = data ? data->LookupForm<RE::TESRace>(local, plugin) : nullptr;
				if (!race) {
					logs::info("{}: no race {:X} in {} (plugin not loaded?)", a_animal.name, local, plugin);
					continue;
				}
				if (FindAnimal(race)) {
					continue;
				}
				Animal animal = a_animal;
				animal.race = race;
				animals.push_back(animal);
				logs::info("{}: race {} ({:08X}) from {}, size {:.2f}, girth {:.2f}, belly {:.1f}{}", animal.name, race->GetFormEditorID(),
					race->GetFormID(), plugin, animal.size, animal.girth, animal.belly, animal.piglet ? ", piglet style" : "");
			}
		}

		// one animal file: [Animal] sName, sRaces, fSize, fGirth, fBelly
		void LoadAnimal(const std::filesystem::path& a_path)
		{
			std::ifstream file(a_path);
			std::string   line;
			std::string   races;
			Animal        animal;
			Head          head;
			animal.name = a_path.stem().string();
			while (std::getline(file, line)) {
				line = Trim(line);
				const auto equals = line.find('=');
				if (line.empty() || line[0] == ';' || line[0] == '#' || line[0] == '[' || equals == std::string::npos) {
					continue;
				}
				std::string field = Trim(line.substr(0, equals));
				std::string value = Trim(line.substr(equals + 1));
				if (const auto comment = value.find(" ;"); comment != std::string::npos) {
					value = Trim(value.substr(0, comment));
				}
				std::ranges::transform(field, field.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
				auto number = [&](float& a_out) {
					try {
						a_out = std::stof(value);
					} catch (...) {
						logs::warn("{}: bad number '{}' for {}", a_path.filename().string(), value, field);
					}
				};
				if (field == "sname") {
					animal.name = value;
				} else if (field == "sraces") {
					races = value;
				} else if (field == "fsize") {
					number(animal.size);
				} else if (field == "fgirth") {
					number(animal.girth);
				} else if (field == "fbelly") {
					number(animal.belly);
				} else if (field == "sstyle") {
					std::string style = value;
					std::ranges::transform(style, style.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
					animal.piglet = style == "piglet";
				} else if (field == "fheadturn") {
					number(head.turn);
				} else if (field == "fheadpitch") {
					number(head.pitch);
				} else if (field == "fheadyaw") {
					number(head.yaw);
				} else if (field == "fheadroll") {
					number(head.roll);
				} else if (field == "fnecklift") {
					number(head.neck);
				}
			}
			animal.size = std::clamp(animal.size, 0.2f, 3.0f);
			animal.girth = std::clamp(animal.girth, 0.3f, 3.0f);
			head.turn = std::clamp(head.turn, 0.0f, 1.0f);
			head.neck = std::clamp(head.neck, 0.0f, 2.0f);
			animal.head = heads.size();
			heads.push_back({ animal.name, a_path, head, animal.piglet });
			AddRaces(races, animal);
		}
	}

	void Init()
	{
		animals.clear();
		heads.clear();
		std::error_code error;
		const std::filesystem::path folder{ "Data/SKSE/Plugins/CatCarry/Animals" };
		std::vector<std::filesystem::path> files;
		for (const auto& entry : std::filesystem::directory_iterator(folder, error)) {
			if (entry.is_regular_file() && entry.path().extension() == ".ini") {
				files.push_back(entry.path());
			}
		}
		std::ranges::sort(files);
		for (const auto& path : files) {
			LoadAnimal(path);
		}
		AddRaces(Settings::Get().races, Animal{ .name = "CatCarry.ini" });
		if (animals.empty()) {
			logs::warn("No animals to pick up: install House Cats (mihailhousecat.esp) and the matching FOMOD options");
		}
	}

	std::vector<AnimalHead>& Heads()
	{
		return heads;
	}

	void SaveHeads()
	{
		for (const auto& entry : heads) {
			std::vector<std::string> lines;
			{
				std::ifstream file(entry.file);
				std::string   line;
				while (std::getline(file, line)) {
					lines.push_back(line);
				}
			}
			const auto& h = entry.head;
			const std::array<std::pair<const char*, float>, 5> values{ { { "fHeadTurn", h.turn }, { "fHeadPitch", h.pitch }, { "fHeadYaw", h.yaw },
				{ "fHeadRoll", h.roll }, { "fNeckLift", h.neck } } };
			for (const auto& [setting, value] : values) {
				const std::string text = std::format("{} = {}", setting, std::round(value * 100.0f) / 100.0f);
				auto              lower = [](std::string a_text) {
                    std::ranges::transform(a_text, a_text.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                    return a_text;
				};
				const std::string settingLower = lower(setting);
				auto              it = std::ranges::find_if(lines, [&](const std::string& a_line) {
                    const auto equals = a_line.find('=');
                    return equals != std::string::npos && lower(Trim(a_line.substr(0, equals))) == settingLower;
                });
				if (it != lines.end()) {
					*it = text;
				} else {
					lines.push_back(text);
				}
			}
			std::ofstream file(entry.file, std::ios::trunc);
			for (const auto& line : lines) {
				file << line << "\n";
			}
			if (!file) {
				logs::warn("Couldn't write {}", entry.file.string());
			}
		}
	}

	bool IsCat(RE::TESObjectREFR* a_ref)
	{
		auto* actor = a_ref ? a_ref->As<RE::Actor>() : nullptr;
		auto* race = actor ? actor->GetRace() : nullptr;
		return race && FindAnimal(race);
	}

	bool CanPickUp(RE::TESObjectREFR* a_ref)
	{
		const auto& s = Settings::Get();
		auto*       player = Player();
		if (!s.enabled || state.phase != Phase::kIdle || !player || !IsCat(a_ref)) {
			return false;
		}
		auto* cat = a_ref->As<RE::Actor>();
		if (cat->IsDead() || cat->IsDisabled() || !cat->Get3D() || !player->Get3D(false)) {
			return false;
		}
		if (M::Length(Flat(cat->GetPosition() - player->GetPosition())) > s.maxDistance) {
			return false;
		}
		return !PlayerBusy(player, false);
	}

	bool TryBegin(RE::TESObjectREFR* a_ref)
	{
		auto* player = Player();
		if (!CanPickUp(a_ref) || PlayerBusy(player, true)) {
			return false;
		}
		auto* cat = a_ref->As<RE::Actor>();
		state = {};
		state.cat = cat->GetHandle();
		if (const auto* animal = FindAnimal(cat->GetRace()); animal) {
			state.animalSize = animal->size;
			state.animalGirth = animal->girth;
			state.animalBelly = animal->belly;
			state.piglet = animal->piglet;
			state.head = animal->head;
		}
		FreezeCat(cat);

		auto* camera = RE::PlayerCamera::GetSingleton();
		state.wasFirstPerson = camera && camera->IsInFirstPerson();
		if (state.wasFirstPerson) {
			camera->ForceThirdPerson();
		}
		DisableControls(kAlwaysOff);
		Lock(player);
		BlockKeys(true);
		if (player->AsActorState()->GetWeaponState() != RE::WEAPON_STATE::kSheathed) {
			player->DrawWeaponMagicHands(false);
			state.phase = Phase::kSheathe;
			state.time = 0.0f;
		} else {
			StartReach();
		}
		return true;
	}

	void Update()
	{
		auto* player = Player();
		if (!player) {
			return;
		}
		if (state.phase == Phase::kIdle) {
			const float delta = Delta();
			if (watchAfter > 0.0f) {
				watchAfter -= delta;
				Unstick(player, delta, drivenAfter, "After the cat");
				KeepControlsOn("After the cat");
			}
			return;
		}
		const float delta = Delta();
		if (state.phase == Phase::kHold) {
			Unstick(player, delta, state.drivenFor, "Holding the cat");
			KeepControlsOn("Holding the cat");
		}
		if (!StillFine(player)) {
			return;
		}
		KeepCat(player, delta);
		Step(player, delta);
		KeepStill(player);
		if (state.phase != Phase::kIdle) {
			KeepThirdPerson();
		}
		if (state.phase == Phase::kIdle || state.phase == Phase::kSheathe || state.phase == Phase::kStand) {
			return;
		}
		if (!Body::Ready() && !Body::Setup(player)) {
			Drop("your arms weren't found");
			return;
		}
		Body::BeginFrame();
		ApplyPose(player, delta);
	}

	bool InterceptActivate(RE::TESObjectREFR* a_ref)
	{
		if (key.replaying || !a_ref) {
			return false;
		}
		// the cat being picked up / carried: its activations go nowhere
		if (Active() && Cat() && a_ref == Cat()) {
			return true;
		}
		if (!CanPickUp(a_ref)) {
			return false;
		}
		if (key.fired) {
			return true;
		}
		if (key.down) {
			// sent on the press: wait and see whether it's a tap or a hold
			key.heldBack = a_ref->GetHandle();
			return true;
		}
		return false;
	}

	void OnActivateKey(bool a_pressed, float a_heldFor)
	{
		if (a_pressed && a_heldFor <= 0.0f) {
			key = {};
			key.down = true;
			return;
		}
		if (a_pressed) {
			if (!key.down || key.fired || a_heldFor < Settings::Get().holdTime) {
				return;
			}
			key.fired = true;
			key.heldBack = {};
			if (auto* ui = RE::UI::GetSingleton(); ui && (ui->GameIsPaused() || ui->IsMenuOpen(RE::Console::MENU_NAME))) {
				return;
			}
			if (state.phase == Phase::kHold) {
				state.putDownRequest = true;
				return;
			}
			if (Active()) {
				return;
			}
			auto* pick = RE::CrosshairPickData::GetSingleton();
			auto  target = pick ? pick->GetActiveTarget().get() : nullptr;
			if (target && IsCat(target.get())) {
				TryBegin(target.get());
			}
			return;
		}
		// released: a tap that we held back gets its activation now
		const auto heldBack = key.heldBack;
		const bool fired = key.fired;
		key = {};
		if (!fired && state.phase == Phase::kHold) {
			ActivateDoor();
			return;
		}
		if (!fired && heldBack) {
			SKSE::GetTaskInterface()->AddTask([heldBack]() {
				auto ref = heldBack.get();
				auto* player = RE::PlayerCharacter::GetSingleton();
				if (ref && player) {
					key.replaying = true;
					ref->ActivateRef(player, 0, nullptr, 1, false);
					key.replaying = false;
				}
			});
		}
	}

	bool KeepsActivateKey()
	{
		if (Active()) {
			return true;
		}
		if (!Settings::Get().enabled) {
			return false;
		}
		auto* pick = RE::CrosshairPickData::GetSingleton();
		auto  target = pick ? pick->GetActiveTarget().get() : nullptr;
		return target && IsCat(target.get());
	}

	bool Active()
	{
		return state.phase != Phase::kIdle;
	}

	void Reset()
	{
		if (state.phase != Phase::kIdle) {
			logs::info("Game loaded while holding a cat; starting over");
		}
		EnableControls(static_cast<UEFlag>(0xFFFFFFFF));
		BlockKeys(false);
		Body::Release();
		CatPose::Release();
		state = {};
		watchAfter = 0.0f;
	}

	RE::FormID HeldCat()
	{
		auto* cat = Cat();
		return cat && state.catHidden ? cat->GetFormID() : 0;
	}

	void RestoreCat(RE::FormID a_cat)
	{
		auto* cat = RE::TESForm::LookupByID<RE::Actor>(a_cat);
		auto* player = Player();
		if (!cat || !player) {
			return;
		}
		logs::info("The save was made while holding {}; putting it next to you", cat->GetDisplayFullName());
		cat->MoveTo(player);
		if (cat->IsDisabled()) {
			cat->Enable(false);
		}
		if (auto* root = cat->Get3D(); root) {
			root->SetAppCulled(false);
		}
		if (!cat->IsAIEnabled()) {
			cat->EnableAI(true);
		}
	}
}
