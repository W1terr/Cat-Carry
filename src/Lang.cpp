#include "Lang.h"

#include <filesystem>
#include <fstream>
#include <shared_mutex>
#include <unordered_map>

namespace Lang
{
	namespace
	{
		constexpr auto kFolder = "Data/SKSE/Plugins/CatCarry/lang";

		constexpr Language kLanguages[]{
			{ "english", "English", nullptr },
			{ "french", "Fran\xC3\xA7"
			            "ais",
				nullptr },
			{ "german", "Deutsch", nullptr },
			{ "italian", "Italiano", nullptr },
			{ "spanish", "Espa\xC3\xB1ol", nullptr },
			{ "polish", "Polski", "Polish" },
			{ "russian", "\xD0\xA0\xD1\x83\xD1\x81\xD1\x81\xD0\xBA\xD0\xB8\xD0\xB9 (Russian)", "Cyrillic" },
			{ "japanese", "\xE6\x97\xA5\xE6\x9C\xAC\xE8\xAA\x9E (Japanese)", "Japanese" },
			{ "chinese", "\xE7\xB9\x81\xE9\xAB\x94\xE4\xB8\xAD\xE6\x96\x87 (Chinese)", "Chinese" },
		};

		struct StringHash
		{
			using is_transparent = void;
			std::size_t operator()(std::string_view a_str) const { return std::hash<std::string_view>{}(a_str); }
		};

		std::atomic<const Language*>                                              current{ &kLanguages[0] };
		std::unordered_map<std::string, std::string, StringHash, std::equal_to<>> table;
		std::shared_mutex                                                         lock;  // Set() against Copy() from other threads

		void LoadTable(const Language& a_language)
		{
			std::unique_lock guard(lock);
			table.clear();
			if (&a_language == &kLanguages[0]) {
				return;
			}
			const auto    path = std::filesystem::path{ kFolder } / std::format("{}.json", a_language.id);
			std::ifstream file(path);
			if (!file) {
				logs::error("Missing {}, texts stay in English", path.string());
				return;
			}
			try {
				const auto j = json::parse(file, nullptr, true, true);
				for (const auto& [english, text] : j.items()) {
					if (text.is_string() && !text.get_ref<const std::string&>().empty()) {
						table.emplace(english, text.get<std::string>());
					}
				}
				logs::info("Loaded {} texts from {}", table.size(), path.string());
			} catch (const std::exception& e) {
				logs::error("Failed to read {}: {}", path.string(), e.what());
				table.clear();
			}
		}

		const char* Find(const char* a_english)
		{
			if (!table.empty()) {
				if (const auto it = table.find(std::string_view(a_english)); it != table.end()) {
					return it->second.c_str();
				}
			}
			return a_english;
		}
	}

	std::span<const Language> All()
	{
		return kLanguages;
	}

	const Language& Current()
	{
		return *current.load();
	}

	const Language& Game()
	{
		auto*       ini = RE::INISettingCollection::GetSingleton();
		auto*       setting = ini ? ini->GetSetting("sLanguage:General") : nullptr;
		std::string id = setting && setting->GetType() == RE::Setting::Type::kString && setting->GetString() ? setting->GetString() : "";
		std::ranges::transform(id, id.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
		const auto it = std::ranges::find_if(kLanguages, [&](const Language& a_language) { return id == a_language.id; });
		return it != std::end(kLanguages) ? *it : kLanguages[0];
	}

	void Set(std::string_view a_id)
	{
		const auto it = std::ranges::find_if(kLanguages, [&](const Language& a_language) { return a_id == a_language.id; });
		current = it != std::end(kLanguages) ? &*it : &kLanguages[0];
		LoadTable(*current.load());
	}

	const char* T(const char* a_english)
	{
		return Find(a_english);
	}

	std::string Copy(const char* a_english)
	{
		std::shared_lock guard(lock);
		return Find(a_english);
	}
}
