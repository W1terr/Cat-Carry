#include "Settings.h"

#include <fstream>
#include <map>
#include <sstream>

namespace Settings
{
	namespace
	{
		constexpr auto kPath = "Data/SKSE/Plugins/CatCarry.ini";

		Values values;

		std::string Trim(std::string a_text)
		{
			const auto first = a_text.find_first_not_of(" \t\r\n");
			if (first == std::string::npos) {
				return {};
			}
			const auto last = a_text.find_last_not_of(" \t\r\n");
			return a_text.substr(first, last - first + 1);
		}

		std::string Lower(std::string a_text)
		{
			std::ranges::transform(a_text, a_text.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
			return a_text;
		}

		using Table = std::map<std::string, std::string>;  // "section.key" (lower case) -> value

		Table Read()
		{
			Table         table;
			std::ifstream file(kPath);
			std::string   line;
			std::string   section;
			while (std::getline(file, line)) {
				line = Trim(line);
				if (line.empty() || line[0] == ';' || line[0] == '#') {
					continue;
				}
				if (line.front() == '[' && line.back() == ']') {
					section = Lower(Trim(line.substr(1, line.size() - 2)));
					continue;
				}
				const auto equals = line.find('=');
				if (equals == std::string::npos) {
					continue;
				}
				std::string value = Trim(line.substr(equals + 1));
				if (const auto comment = value.find(" ;"); comment != std::string::npos) {
					value = Trim(value.substr(0, comment));
				}
				table[section + "." + Lower(Trim(line.substr(0, equals)))] = value;
			}
			return table;
		}

		void Get(const Table& a_table, const char* a_key, bool& a_value)
		{
			if (auto it = a_table.find(Lower(a_key)); it != a_table.end()) {
				a_value = it->second == "1" || Lower(it->second) == "true";
			}
		}

		void Get(const Table& a_table, const char* a_key, int& a_value)
		{
			if (auto it = a_table.find(Lower(a_key)); it != a_table.end()) {
				try {
					a_value = std::stoi(it->second);
				} catch (...) {
				}
			}
		}

		void Get(const Table& a_table, const char* a_key, float& a_value)
		{
			if (auto it = a_table.find(Lower(a_key)); it != a_table.end()) {
				try {
					a_value = std::stof(it->second);
				} catch (...) {
				}
			}
		}

		void Get(const Table& a_table, const char* a_key, std::string& a_value)
		{
			if (auto it = a_table.find(Lower(a_key)); it != a_table.end()) {
				a_value = it->second;
			}
		}

		std::string Number(float a_value)
		{
			auto text = std::format("{:.2f}", a_value);
			while (text.ends_with('0')) {
				text.pop_back();
			}
			if (text.ends_with('.')) {
				text.pop_back();
			}
			return text == "-0" ? "0" : text;
		}
	}

	Values& Get()
	{
		return values;
	}

	void Load()
	{
		const Table t = Read();
		auto&       v = values;
		Get(t, "General.bEnabled", v.enabled);
		Get(t, "General.fMaxDistance", v.maxDistance);
		Get(t, "General.sLanguage", v.language);
		Get(t, "General.sPickUpText", v.pickUpText);
		Get(t, "General.fHoldTime", v.holdTime);
		Get(t, "General.sRaces", v.races);
		Get(t, "Pickup.bKneel", v.kneel);
		Get(t, "Pickup.fKneelBelow", v.kneelBelow);
		Get(t, "Pickup.fMaxLean", v.maxLean);
		Get(t, "Pickup.bLookAtCat", v.lookAtCat);
		Get(t, "Hold.iSide", v.side);
		Get(t, "Hold.fHeldSize", v.heldSize);
		Get(t, "Hold.fMaxBodyLength", v.maxBodyLength);
		Get(t, "Hold.fCatRight", v.catRight);
		Get(t, "Hold.fCatForward", v.catForward);
		Get(t, "Hold.fCatUp", v.catUp);
		Get(t, "Hold.fCatLean", v.catLean);
		Get(t, "Hold.fCatBack", v.catBack);
		Get(t, "Hold.fCatTurn", v.catTurn);
		Get(t, "Hold.fChestAngle", v.chestAngle);
		Get(t, "Hold.fChestAlong", v.chestAlong);
		Get(t, "Hold.fChestRadius", v.chestRadius);
		Get(t, "Hold.fBottomBelow", v.bottomBelow);
		Get(t, "Hold.fBottomRadius", v.bottomRadius);
		Get(t, "Hold.fFingerCurl", v.fingerCurl);
		Get(t, "Hold.fPawsForward", v.pawsForward);
		Get(t, "Hold.fLegHang", v.legHang);
		Get(t, "Hold.bCatLooksAround", v.catLooksAround);
		Get(t, "Hold.bTailSway", v.tailSway);
		Get(t, "Hold.fHeadAway", v.headAway);
		Get(t, "Hold.bPetCat", v.petCat);
		Get(t, "Hold.fPetEvery", v.petEvery);
		v.side = std::clamp(v.side, 0, 2);
		v.heldSize = std::clamp(v.heldSize, 0.3f, 1.0f);
		v.maxBodyLength = std::max(v.maxBodyLength, 0.0f);
		v.petEvery = std::max(v.petEvery, 1.0f);
		logs::info("Settings loaded from {}", kPath);
	}

	void Save()
	{
		const auto&        v = values;
		std::ostringstream o;
		o << "; Cat Carry settings. The holding pose can be tuned live in SKSE Menu Framework (Cat Carry > Settings).\n\n";
		o << "[General]\n";
		o << "bEnabled = " << v.enabled << "\n";
		o << "; how close (game units, along the ground) you have to be to pick a cat up\n";
		o << "fMaxDistance = " << Number(v.maxDistance) << "\n";
		o << "; english, french, german, italian, spanish, polish, russian, japanese, chinese (empty = the game's language)\n";
		o << "sLanguage = " << v.language << "\n";
		o << "; the word shown before the cat's name when you look at it (empty = \"Pick up (hold)\" in your language)\n";
		o << "sPickUpText = " << v.pickUpText << "\n";
		o << "; seconds you hold Activate to pick the cat up (a short tap stays the game's normal activation)\n";
		o << "fHoldTime = " << Number(v.holdTime) << "\n";
		o << "; more races that can be picked up (plugin|form ID, separated by commas) besides the animals in\n";
		o << "; SKSE\\Plugins\\CatCarry\\Animals\\*.ini (installed by the FOMOD: House Cats, Piglet)\n";
		o << "sRaces = " << v.races << "\n\n";
		o << "[Pickup]\n";
		o << "; kneel down for cats on the floor (else you only bend forward)\n";
		o << "bKneel = " << v.kneel << "\n";
		o << "; kneel when the cat's body is at most this high above your feet\n";
		o << "fKneelBelow = " << Number(v.kneelBelow) << "\n";
		o << "; degrees you may bend forward to reach the cat\n";
		o << "fMaxLean = " << Number(v.maxLean) << "\n";
		o << "bLookAtCat = " << v.lookAtCat << "\n\n";
		o << "[Hold]\n";
		o << "; 0 = the side its head points to when you pick it up, 1 = always on your left, 2 = always on your right\n";
		o << "iSide = " << v.side << "\n";
		o << "; the cat gets a bit smaller in your arms (looks right against your body; big cats of other mods fit too):\n";
		o << "; its size times fHeldSize, and at most fMaxBodyLength long from chest to hips (game units, 0 = no limit;\n";
		o << "; a House Cat is about 22)\n";
		o << "fHeldSize = " << Number(v.heldSize) << "\n";
		o << "fMaxBodyLength = " << Number(v.maxBodyLength) << "\n";
		o << "; the cat's body center from your chest, for the left side (mirrored on the right): right, forward, up\n";
		o << "fCatRight = " << Number(v.catRight) << "\n";
		o << "fCatForward = " << Number(v.catForward) << "\n";
		o << "fCatUp = " << Number(v.catUp) << "\n";
		o << "; degrees: head tipped out to the side, head tipped back towards you, belly turned outwards\n";
		o << "fCatLean = " << Number(v.catLean) << "\n";
		o << "fCatBack = " << Number(v.catBack) << "\n";
		o << "fCatTurn = " << Number(v.catTurn) << "\n";
		o << "; the cat's upper front legs: 0 = hang straight down, higher = forward over your arm (the paws hang)\n";
		o << "fPawsForward = " << Number(v.pawsForward) << "\n";
		o << "; 0 = its legs keep the pose they had on the floor .. 1 = they hang fully\n";
		o << "fLegHang = " << Number(v.legHang) << "\n";
		o << "bCatLooksAround = " << v.catLooksAround << "\n";
		o << "bTailSway = " << v.tailSway << "\n";
		o << "; degrees your head turns and tilts away from the cat's head while you hold it\n";
		o << "fHeadAway = " << Number(v.headAway) << "\n";
		o << "; your hand on its chest strokes the cat now and then, every fPetEvery seconds\n";
		o << "bPetCat = " << v.petCat << "\n";
		o << "fPetEvery = " << Number(v.petEvery) << "\n";

		std::ofstream file(kPath, std::ios::trunc);
		file << o.str();
		if (!file) {
			logs::warn("Couldn't write {}", kPath);
		}
	}
}
