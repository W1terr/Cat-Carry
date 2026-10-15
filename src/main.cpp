#include "Carry.h"
#include "Hooks.h"
#include "Lang.h"
#include "Menu.h"
#include "Settings.h"

namespace
{
	constexpr std::uint32_t kSaveID = 'CCRY';
	constexpr std::uint32_t kHeldCat = 'HELD';
	constexpr std::uint32_t kVersion = 1;

	// a cat that was in your arms when the game was saved (frozen and parked behind you in that save)
	RE::FormID pendingCat = 0;

	void OnSave(SKSE::SerializationInterface* a_intfc)
	{
		const RE::FormID cat = Carry::HeldCat();
		if (cat && a_intfc->OpenRecord(kHeldCat, kVersion)) {
			a_intfc->WriteRecordData(cat);
		}
	}

	void OnLoad(SKSE::SerializationInterface* a_intfc)
	{
		std::uint32_t type, version, length;
		while (a_intfc->GetNextRecordInfo(type, version, length)) {
			RE::FormID cat = 0;
			if (type == kHeldCat && a_intfc->ReadRecordData(cat) && a_intfc->ResolveFormID(cat, cat)) {
				pendingCat = cat;
			}
		}
	}

	void OnRevert(SKSE::SerializationInterface*)
	{
		pendingCat = 0;
	}

	void OnMessage(SKSE::MessagingInterface::Message* a_msg)
	{
		switch (a_msg->type) {
		case SKSE::MessagingInterface::kDataLoaded:
			Settings::Load();
			Lang::Set(Settings::Get().language.empty() ? Lang::Game().id : Settings::Get().language);
			Carry::Init();
			Hooks::Install();
			Menu::Register();
			break;
		case SKSE::MessagingInterface::kPreLoadGame:
		case SKSE::MessagingInterface::kNewGame:
			Carry::Reset();
			break;
		case SKSE::MessagingInterface::kPostLoadGame:
			if (pendingCat) {
				Carry::RestoreCat(pendingCat);
				pendingCat = 0;
			}
			break;
		default:
			break;
		}
	}
}

SKSE_PLUGIN_LOAD(const SKSE::LoadInterface* a_skse)
{
	SKSE::Init(a_skse, { .trampoline = true, .trampolineSize = 64 });
	logs::info("Cat Carry loading");
	SKSE::GetMessagingInterface()->RegisterListener(OnMessage);
	if (auto* serialization = SKSE::GetSerializationInterface(); serialization) {
		serialization->SetUniqueID(kSaveID);
		serialization->SetSaveCallback(OnSave);
		serialization->SetLoadCallback(OnLoad);
		serialization->SetRevertCallback(OnRevert);
	}
	return true;
}
