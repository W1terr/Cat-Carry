#include "Menu.h"

#include "Framework.h"
#include "Lang.h"
#include "Settings.h"

namespace Menu
{
	namespace
	{
		using namespace ImGuiMCP;
		using Lang::F;
		using Lang::T;

		// translated label with a fixed ImGui id (the id must not change with the language or repeat on the page)
		std::string Id(const char* a_english)
		{
			return std::format("{}###{}", T(a_english), a_english);
		}

		void Hint(const char* a_english)
		{
			SameLine();
			TextDisabled("(?)");
			if (IsItemHovered()) {
				SetTooltip("%s", T(a_english));
			}
		}

		void __stdcall OnEvent(SKSEMenuFramework::Model::EventType a_type)
		{
			if (a_type == SKSEMenuFramework::Model::EventType::kCloseMenu) {
				Settings::Save();
			}
		}

		// "Game language (...)" first, then every language
		void LanguageCombo()
		{
			auto&       s = Settings::Get();
			const auto  languages = Lang::All();
			std::string gameLabel = F("Game language ({})", Lang::Game().name);
			int         index = 0;
			for (std::size_t i = 0; i < languages.size(); ++i) {
				if (s.language == languages[i].id) {
					index = static_cast<int>(i) + 1;
				}
			}
			std::vector<const char*> names{ gameLabel.c_str() };
			for (const auto& language : languages) {
				names.push_back(language.name);
			}
			if (Combo(Id("Language").c_str(), &index, names.data(), static_cast<int>(names.size()))) {
				s.language = index > 0 ? languages[index - 1].id : "";
				Lang::Set(index > 0 ? s.language : Lang::Game().id);
			}
			// without the right glyphs the framework draws "?" for every letter, so this is also shown in English
			if (const auto glyphs = Lang::Current().glyphs) {
				constexpr auto kGlyphHint = "If you see ? instead of letters: in SKSE Menu Framework open Options > Open Settings and set Character Glyphs to {}.";
				TextWrapped("%s", F(kGlyphHint, glyphs).c_str());
				TextDisabled("%s", std::vformat(kGlyphHint, std::make_format_args(glyphs)).c_str());
			}
		}

		void __stdcall Render()
		{
			auto& s = Settings::Get();
			LanguageCombo();

			SeparatorText(T("General"));
			Checkbox(Id("Pick up cats").c_str(), &s.enabled);
			SliderFloat(Id("Reach distance").c_str(), &s.maxDistance, 40.0f, 200.0f, "%.0f");
			Hint("How close (game units, along the ground) you have to be to pick a cat up");
			SliderFloat(Id("Hold Activate for").c_str(), &s.holdTime, 0.15f, 1.5f, T("%.2f s"));
			Hint("Holding Activate this long picks the cat up; a short tap stays the game's normal activation");

			SeparatorText(T("Picking up"));
			Checkbox(Id("Kneel for cats on the floor").c_str(), &s.kneel);
			SliderFloat(Id("Kneel when lower than").c_str(), &s.kneelBelow, 0.0f, 120.0f, "%.0f");
			Hint("Height of the cat's body above your feet");
			SliderFloat(Id("Most you bend forward").c_str(), &s.maxLean, 0.0f, 60.0f, "%.0f\xC2\xB0");
			Checkbox(Id("Look at the cat").c_str(), &s.lookAtCat);

			SeparatorText(T("Holding (changes show right away)"));
			const char* sides[] = { T("Where its head points"), T("Always left"), T("Always right") };
			Combo(Id("Held on").c_str(), &s.side, sides, 3);
			SliderFloat(Id("Cat size in your arms").c_str(), &s.heldSize, 0.3f, 1.0f, "%.2f");
			Hint("The cat shrinks to this much of its own size while you hold it, and grows back when you put it down");
			SliderFloat(Id("Largest cat in your arms").c_str(), &s.maxBodyLength, 0.0f, 40.0f, "%.0f");
			Hint("Bigger cats shrink further until they are at most this long from chest to hips (game units; 0 = no limit). A House Cat is about 22");
			SliderFloat(Id("Cat right").c_str(), &s.catRight, -30.0f, 30.0f, "%.1f");
			Hint("From your chest; for the left side, mirrored on the right");
			SliderFloat(Id("Cat forward").c_str(), &s.catForward, -5.0f, 40.0f, "%.1f");
			SliderFloat(Id("Cat up").c_str(), &s.catUp, -30.0f, 30.0f, "%.1f");
			SliderFloat(Id("Cat tipped outwards").c_str(), &s.catLean, -60.0f, 60.0f, "%.0f\xC2\xB0");
			SliderFloat(Id("Cat tipped back").c_str(), &s.catBack, -60.0f, 60.0f, "%.0f\xC2\xB0");
			SliderFloat(Id("Belly turned outwards").c_str(), &s.catTurn, -90.0f, 90.0f, "%.0f\xC2\xB0");
			SliderFloat(Id("Cat's front legs forward").c_str(), &s.pawsForward, 0.0f, 3.0f, "%.2f");
			Hint("0 = they hang straight down, higher = the upper legs reach forward over your arm and the paws hang");
			SliderFloat(Id("Cat's legs hang").c_str(), &s.legHang, 0.0f, 1.0f, "%.2f");
			Hint("0 = its legs keep the pose they had on the floor, 1 = they hang down fully");
			Checkbox(Id("Cat looks around").c_str(), &s.catLooksAround);
			Checkbox(Id("Tail sways").c_str(), &s.tailSway);
			SliderFloat(Id("Your head away from the cat").c_str(), &s.headAway, 0.0f, 40.0f, "%.0f\xC2\xB0");
			Hint("Your head turns and tilts away so it doesn't go into the cat's head");
			Checkbox(Id("Pet the cat").c_str(), &s.petCat);
			SliderFloat(Id("Pet it every").c_str(), &s.petEvery, 2.0f, 30.0f, T("%.0f s"));
			Hint("Your hand on its chest strokes it twice, then holds it again");
			SeparatorText(T("Animals held in one arm (chickens, Squelch, Little Buddies)"));
			SliderFloat(Id("Hand right").c_str(), &s.handRight, -30.0f, 30.0f, "%.1f");
			Hint("Where your hand holds it, from your chest (for the left side, mirrored on the right): moves your hand and the animal together");
			SliderFloat(Id("Hand forward").c_str(), &s.handForward, 0.0f, 40.0f, "%.1f");
			SliderFloat(Id("Hand up").c_str(), &s.handUp, -30.0f, 20.0f, "%.1f");
			SliderFloat(Id("Animal right").c_str(), &s.animalRight, -20.0f, 20.0f, "%.1f");
			Hint("Moves only the animal in your hand, your hand stays");
			SliderFloat(Id("Animal forward").c_str(), &s.animalForward, -20.0f, 20.0f, "%.1f");
			SliderFloat(Id("Animal up").c_str(), &s.animalUp, -20.0f, 20.0f, "%.1f");
			SliderFloat(Id("Animal turned").c_str(), &s.animalTurn, -180.0f, 180.0f, "%.0f\xC2\xB0");
			Hint("Turns only the animal round its middle, your hand stays: round the vertical, + = towards your middle");
			SliderFloat(Id("Animal tipped").c_str(), &s.animalTip, -180.0f, 180.0f, "%.0f\xC2\xB0");
			Hint("+ = its top away from you, - = towards you");
			SliderFloat(Id("Animal tilted").c_str(), &s.animalTilt, -180.0f, 180.0f, "%.0f\xC2\xB0");
			Hint("+ = its top out to your side, - = towards your middle");
			if (Button(Id("Defaults for the holding pose").c_str())) {
				const Settings::Values defaults;
				s.side = defaults.side;
				s.heldSize = defaults.heldSize;
				s.maxBodyLength = defaults.maxBodyLength;
				s.catRight = defaults.catRight;
				s.catForward = defaults.catForward;
				s.catUp = defaults.catUp;
				s.catLean = defaults.catLean;
				s.catBack = defaults.catBack;
				s.catTurn = defaults.catTurn;
				s.pawsForward = defaults.pawsForward;
				s.legHang = defaults.legHang;
				s.headAway = defaults.headAway;
				s.handRight = defaults.handRight;
				s.handForward = defaults.handForward;
				s.handUp = defaults.handUp;
				s.animalRight = defaults.animalRight;
				s.animalForward = defaults.animalForward;
				s.animalUp = defaults.animalUp;
				s.animalTurn = defaults.animalTurn;
				s.animalTip = defaults.animalTip;
				s.animalTilt = defaults.animalTilt;
			}
		}
	}

	void Register()
	{
		if (!SKSEMenuFramework::IsInstalled() || !GetModuleHandleW(L"SKSEMenuFramework.dll")) {
			logs::info("SKSE Menu Framework not found; settings only in CatCarry.ini");
			return;
		}
		SKSEMenuFramework::SetSection("Cat Carry");
		SKSEMenuFramework::AddSectionItem("Settings", Render);
		SKSEMenuFramework::AddEvent(OnEvent, 0.0f);
	}
}
