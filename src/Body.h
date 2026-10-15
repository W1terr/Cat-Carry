#pragma once

// Procedural pose layer on the player's 3rd person skeleton: spine lean, head look, two-bone arm IK with hand
// rotation, forearm twist, finger curl. Applied after the game animated and updated the skeleton each frame.
namespace Body
{
	enum Side : int
	{
		kLeft = 0,
		kRight = 1
	};

	struct HandGoal
	{
		RE::NiPoint3  palm;        // world point the palm center should touch
		RE::NiMatrix3 rotation;    // world rotation of the hand bone (+Z fingers, -Y palm normal)
		RE::NiPoint3  pole;        // world direction the elbow should point to
		float         weight{ 0.0f };
		float         curl{ 0.0f };  // 0 open .. 1 fist
	};

	struct Goals
	{
		float                   lean{ 0.0f };  // radians, forward about the body's right axis
		RE::NiPoint3            right{ 1.0f, 0.0f, 0.0f };
		std::optional<RE::NiPoint3> lookAt;
		float                   lookWeight{ 0.0f };
		std::optional<RE::NiMatrix3> headTurn;  // world rotation of neck + head before the look (head out of the cat's way)
		std::array<HandGoal, 2> hands;
	};

	// finds the bones of the player's 3rd person skeleton; false if the arms aren't there
	bool Setup(RE::Actor* a_player);
	// puts every bone and skin pointer back the way the game left them
	void Release();
	bool Ready();

	// once per frame before Apply: takes back our rotations the animation didn't replace
	void BeginFrame();
	void Apply(const Goals& a_goals);

	// animated (this frame, before Apply) positions
	RE::NiPoint3 Shoulder(Side a_side);
	RE::NiPoint3 Spine2();
	// animated world rotation of the chest bone (Spine2), if the skeleton has it
	std::optional<RE::NiMatrix3> ChestRotation();
	RE::NiPoint3 Head();
	float        ArmLength(Side a_side);

	// lean (radians, up to a_max) the shoulders need so both palms can reach their goals
	float LeanFor(const Goals& a_goals, float a_max);
}
