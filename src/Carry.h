#pragma once

#include <filesystem>

// Picking a cat up, holding it in your arms and putting it down again.
namespace Carry
{
	// how an animal holds its head in your arms (per animal file, tuned live in SKSE Menu Framework)
	struct Head
	{
		float turn{ 1.0f };   // how far it follows its look around: 0 = in line with its body .. 1 = full
		float pitch{ 0.0f };  // degrees: + nose up
		float yaw{ 0.0f };    // degrees: to one side (mirrored when it's held on your other side)
		float roll{ 0.0f };   // degrees: tilted (mirrored like yaw)
		float neck{ 1.0f };   // how much the neck lifts the head: 0 none .. 2 double
	};

	struct AnimalHead
	{
		std::string           name;
		std::filesystem::path file;  // its animal file (saved back there)
		Head                  head;
		bool                  piglet{ false };  // not shown in the menu
	};

	// one per animal file, editable while the game runs
	std::vector<AnimalHead>& Heads();
	// writes the head values back into the animal files
	void SaveHeads();

	// after the game data loaded: finds the cat races from the settings
	void Init();

	bool IsCat(RE::TESObjectREFR* a_ref);
	// "Pick up" should show for this ref now
	bool CanPickUp(RE::TESObjectREFR* a_ref);
	// the player activated a ref: true if we took the activation (picking the cat up)
	bool TryBegin(RE::TESObjectREFR* a_ref);

	// every frame after the player's skeleton was animated and updated
	void Update();
	// the game activates a ref for the player: true if we keep that activation back (a cat: tap or hold?)
	bool InterceptActivate(RE::TESObjectREFR* a_ref);
	// every Activate key event (any device): holding it picks a cat up / puts it down, a tap opens doors meanwhile
	void OnActivateKey(bool a_pressed, float a_heldFor);

	bool Active();
	// a game is loaded / started: forget everything
	void Reset();

	// co-save: the cat that is held right now, 0 if none
	RE::FormID HeldCat();
	// after loading a save made while holding a cat: brings it back next to the player
	void RestoreCat(RE::FormID a_cat);
}
