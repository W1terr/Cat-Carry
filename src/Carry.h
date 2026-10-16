#pragma once

// Picking a cat (or another small animal) up, holding it in your arms and putting it down again.
namespace Carry
{
	// after the game data loaded: finds the animals' races from the animal files and CatCarry.ini
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
	// the game's activate handler shouldn't see the Activate key held now (you look at a pet or carry one)
	bool KeepsActivateKey();

	bool Active();
	// a game is loaded / started: forget everything
	void Reset();

	// co-save: the cat that is held right now, 0 if none
	RE::FormID HeldCat();
	// after loading a save made while holding a cat: brings it back next to the player
	void RestoreCat(RE::FormID a_cat);
}
