#include "Hooks.h"

#include "Carry.h"
#include "Lang.h"
#include "Settings.h"

namespace Hooks
{
	namespace
	{
		bool skeletonHook = false;

		// The player's 3D update (SE 39446 / AE 40522) animates and updates the 3rd person body, then calls
		// NiAVObject::Update on the 1st person skeleton (+0xD7, this call). Our pose goes on right after it.
		// Other mods (Improved Camera, Item Inspection) hook the same call; the hooks chain.
		struct PlayerSkeletonsUpdate
		{
			static void thunk(RE::NiAVObject* a_object, RE::NiUpdateData* a_data)
			{
				func(a_object, a_data);
				Carry::Update();
			}
			static inline REL::Relocation<decltype(thunk)> func;
		};

		// fallback when the call above isn't where we expect it: the 3rd person camera update
		struct ThirdPersonUpdate
		{
			static void thunk(RE::ThirdPersonState* a_this, RE::BSTSmartPointer<RE::TESCameraState>& a_next)
			{
				func(a_this, a_next);
				if (!skeletonHook) {
					Carry::Update();
				}
			}
			static inline REL::Relocation<decltype(thunk)> func;
		};

		// activating a cat picks it up
		struct NPCActivate
		{
			static bool thunk(RE::TESNPC* a_this, RE::TESObjectREFR* a_target, RE::TESObjectREFR* a_activator, std::uint8_t a_arg3,
				RE::TESBoundObject* a_object, std::int32_t a_count)
			{
				if (a_activator && a_activator->IsPlayerRef() && Carry::InterceptActivate(a_target)) {
					return true;
				}
				return func(a_this, a_target, a_activator, a_arg3, a_object, a_count);
			}
			static inline REL::Relocation<decltype(thunk)> func;
		};

		// "Pick up <name>" instead of "Talk" when looking at a cat (the game formats it "verb\nname")
		struct NPCActivateText
		{
			static bool thunk(RE::TESNPC* a_this, RE::TESObjectREFR* a_target, RE::BSString& a_text)
			{
				if (Carry::CanPickUp(a_target)) {
					const auto& custom = Settings::Get().pickUpText;
					a_text = std::format("{}\n{}", custom.empty() ? Lang::Copy("Pick up (hold)") : custom, a_target->GetDisplayFullName()).c_str();
					return true;
				}
				return func(a_this, a_target, a_text);
			}
			static inline REL::Relocation<decltype(thunk)> func;
		};

		class InputSink : public RE::BSTEventSink<RE::InputEvent*>
		{
		public:
			static InputSink* GetSingleton()
			{
				static InputSink sink;
				return &sink;
			}

			RE::BSEventNotifyControl ProcessEvent(RE::InputEvent* const* a_events, RE::BSTEventSource<RE::InputEvent*>*) override
			{
				if (!a_events) {
					return RE::BSEventNotifyControl::kContinue;
				}
				const auto* userEvents = RE::UserEvents::GetSingleton();
				auto*       controls = RE::ControlMap::GetSingleton();
				for (auto* event = *a_events; event; event = event->next) {
					auto* button = event->AsButtonEvent();
					if (!button) {
						continue;
					}
					// by its name, or by the key bound to Activate (with the Activate control switched off while you
					// carry the cat, the game may send the key without the name)
					bool activate = button->QUserEvent() == userEvents->activate;
					if (!activate && controls) {
						const auto key = controls->GetMappedKey(userEvents->activate.c_str(), button->GetDevice());
						activate = key != 0xFF && key != 0xFFFFFFFF && key == button->GetIDCode();
					}
					if (!activate) {
						continue;
					}
					Carry::OnActivateKey(button->IsPressed(), button->HeldDuration());
				}
				return RE::BSEventNotifyControl::kContinue;
			}
		};

		void InstallSkeletonHook()
		{
			const auto address = REL::RelocationID(39446, 40522).address() + 0xD7;
			if (*reinterpret_cast<const std::uint8_t*>(address) != 0xE8) {
				logs::warn("Player skeleton update call not found; posing from the camera update instead");
				return;
			}
			PlayerSkeletonsUpdate::func = SKSE::GetTrampoline().write_call<5>(address, PlayerSkeletonsUpdate::thunk);
			skeletonHook = true;
		}
	}

	void Install()
	{
		InstallSkeletonHook();

		REL::Relocation<std::uintptr_t> thirdPerson{ RE::VTABLE_ThirdPersonState[0] };
		ThirdPersonUpdate::func = thirdPerson.write_vfunc(0x3, ThirdPersonUpdate::thunk);

		REL::Relocation<std::uintptr_t> npc{ RE::VTABLE_TESNPC[0] };
		NPCActivate::func = npc.write_vfunc(0x37, NPCActivate::thunk);
		NPCActivateText::func = npc.write_vfunc(0x4C, NPCActivateText::thunk);

		if (auto* input = RE::BSInputDeviceManager::GetSingleton(); input) {
			// first in line, so the key state is known before the game activates anything on the same event
			input->PrependEventSink(InputSink::GetSingleton());
		}
		logs::info("Hooks installed ({})", skeletonHook ? "player skeleton update" : "camera update");
	}
}
