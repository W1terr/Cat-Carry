#pragma once

// Data\SKSE\Plugins\CatCarry.ini
namespace Settings
{
	struct Values
	{
		// [General]
		bool        enabled{ true };
		float       maxDistance{ 110.0f };  // you to the cat (horizontal) for "Pick up"
		std::string language{};          // Lang id, empty = the game's language
		std::string pickUpText{};        // empty = "Pick up (hold)" in the menu's language
		float       holdTime{ 0.45f };       // seconds Activate has to be held to pick a cat up
		std::string races{};             // more races besides the ones in CatCarry\Animals\*.ini

		// [Pickup]
		bool  kneel{ true };          // kneel down for cats on the floor
		float kneelBelow{ 60.0f };    // cat's body this far above your feet or less: kneel
		float maxLean{ 40.0f };       // degrees you may bend forward to reach
		bool  lookAtCat{ true };

		// [Hold] your body frame, from your chest (spine2): right / forward / up; degrees
		int   side{ 0 };              // 0 the side its head points to, 1 left, 2 right
		float heldSize{ 0.8f };      // the cat shrinks to this much of its size in your arms
		float maxBodyLength{ 18.0f }; // ... and to at most this long chest to hips (game units; 0 = no limit)
		float catRight{ -5.0f };      // cat on your left (mirrored for the right side)
		float catForward{ 17.0f };
		float catUp{ 6.0f };
		float catLean{ 20.0f };       // head tipped out to the side
		float catBack{ 10.0f };       // head tipped back towards you
		float catTurn{ 18.0f };       // belly turned outwards
		float chestAngle{ 135.0f };   // chest hand around the cat's chest: 0 its back, 90 its inner side, 180 its belly
		float chestAlong{ 1.0f };     // chest hand up (+) / down (-) along the cat from its chest
		float chestRadius{ 9.2f };    // chest hand's distance from the cat's spine
		float bottomBelow{ 11.1f };    // other hand this far below the cat's hips
		float bottomRadius{ 2.3f };   // ... and this far out to its belly
		float fingerCurl{ 0.47f };
		float pawsForward{ 1.18f };
		float legHang{ 0.5f };        // 0 = legs keep the pose they had on the floor .. 1 = they hang fully
		bool  catLooksAround{ true };
		bool  tailSway{ true };
		float headAway{ 15.0f };      // degrees your head turns / tilts away from the cat's head while you hold it
		bool  petCat{ true };         // your chest hand strokes the cat now and then
		float petEvery{ 7.0f };       // seconds between two pettings
	};

	Values& Get();
	void    Load();
	void    Save();
}
