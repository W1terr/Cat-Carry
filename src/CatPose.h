#pragma once

// The cat in the player's arms: the real cat's own 3D (the cat is frozen meanwhile), placed every frame and posed by
// us: legs and tail hang down, the head looks around. Works with the cats' skeever skeleton and the cow skeleton (Piglet).
namespace CatPose
{
	// the cat's body in its own root space (root units, before the root's scale)
	struct Frame
	{
		RE::NiPoint3 torso;
		RE::NiPoint3 pelvis;
		RE::NiPoint3 middle;
		RE::NiPoint3 spine;  // pelvis -> torso
		RE::NiPoint3 back;   // up out of its back
		RE::NiPoint3 side;   // its right
		float        halfLength{ 10.0f };
	};

	// extra life for animals that have their own little show (Piglet), all 0 for the cats
	struct Mood
	{
		float tailWag{ 0.0f };  // quick happy wiggle of the tail
		float paddle{ 0.0f };   // legs run in the air (diagonal pairs together, like a trot)
		float sniff{ 0.0f };    // little snout nods, the mouth working
		float ears{ 0.0f };     // -1 laid back .. 1 perked forward
		// head settings of the animal (radians)
		float headPitch{ 0.0f };  // + nose up
		float headYaw{ 0.0f };
		float headRoll{ 0.0f };
		float neck{ 1.0f };       // neck lift amount
	};

	// body frame of a cat's 3D
	Frame Measure(RE::NiAVObject* a_root);

	// starts moving and posing the cat's 3D (remembers the pose it has now)
	bool Hold(RE::Actor* a_cat);
	// gives the bones their own pose back and stops touching the 3D
	void Release();
	bool Active();
	RE::NiAVObject* Root();

	// puts the cat's root at a world transform (scale included) and poses its bones:
	// a_hold 0 = as it was on the ground .. 1 = fully held; a_look = world direction its head turns to;
	// a_pawsForward = how far forward the upper front legs reach (against 0.45 down)
	void Place(const RE::NiTransform& a_world, float a_hold, float a_time, const RE::NiPoint3& a_look, float a_sway, float a_pawsForward, float a_legs,
		const Mood& a_mood = {});
}
