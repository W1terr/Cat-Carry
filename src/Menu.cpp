#include "Menu.h"

#include "Carry.h"
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
				Carry::SaveHeads();
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
			}

			SeparatorText(T("Heads (pick the animal up, changes show right away; saved to its animal file)"));
			bool any = false;
			auto& heads = Carry::Heads();
			for (std::size_t i = 0; i < heads.size(); ++i) {
				auto& entry = heads[i];
				if (entry.piglet) {
					continue;  // tuned for good in Piglet.ini
				}
				any = true;
				PushID(static_cast<int>(i));
				Text("%s", T(entry.name.c_str()));
				SliderFloat(Id("Look around").c_str(), &entry.head.turn, 0.0f, 1.0f, "%.2f");
				Hint("How far the head follows its look around: 0 = in line with its body, 1 = full");
				SliderFloat(Id("Head up / down").c_str(), &entry.head.pitch, -60.0f, 60.0f, "%.0f\xC2\xB0");
				SliderFloat(Id("Head left / right").c_str(), &entry.head.yaw, -60.0f, 60.0f, "%.0f\xC2\xB0");
				Hint("Mirrored when it's held on your other side");
				SliderFloat(Id("Head tilt").c_str(), &entry.head.roll, -45.0f, 45.0f, "%.0f\xC2\xB0");
				SliderFloat(Id("Neck lift").c_str(), &entry.head.neck, 0.0f, 2.0f, "%.2f");
				Hint("How much the neck lifts the head up out of the body line: 0 none, 1 normal, 2 double");
				if (Button(Id("Reset this head").c_str())) {
					entry.head = Carry::Head{};
				}
				PopID();
			}
			if (!any) {
				TextDisabled("%s", T("No animal files loaded"));
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
