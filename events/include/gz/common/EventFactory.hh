/*
 * Copyright (C) 2023 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef GZ_COMMON_EVENTFACTORY_HH_
#define GZ_COMMON_EVENTFACTORY_HH_

#include <cstdint>
#include <cstring>
#include <deque>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/SingletonT.hh>
#include <gz/common/Util.hh>
#include <gz/common/Event.hh>
#include <gz/common/config.hh>

namespace gz
{
namespace common
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_COMMON_VERSION_NAMESPACE {
// namespace events
// {
  /// \brief A base class for an object responsible for creating events.
  class EventDescriptorBase
  {
    /// \brief Destructor
    public: virtual ~EventDescriptorBase() = default;

    /// \brief Create an instance of an Event.
    /// \return Pointer to an event.
    public: virtual std::unique_ptr<Event> Create() const = 0;
  };

  /// \brief A class for an object responsible for creating events.
  /// \tparam EventTypeT type of event to describe.
  template <typename EventTypeT>
  class EventDescriptor
    : public EventDescriptorBase
  {
    /// \brief Documentation inherited
    public: std::unique_ptr<Event> Create() const override
    {
      return std::make_unique<EventTypeT>();
    }
  };

  /// \brief A wrapper around uintptr_t to prevent implicit conversions.
  struct RegistrationObjectId
  {
    /// \brief Construct object from a pointer.
    /// \param[in] _ptr Arbitrary pointer.
    explicit RegistrationObjectId(void *_ptr)
        : id(reinterpret_cast<std::uintptr_t>(_ptr))
    {
    }

    /// \brief Construct object from a uintptr_t.
    /// \param[in] _ptr Arbitrary pointer address.
    explicit RegistrationObjectId(std::uintptr_t _ptrAddress)
        : id(_ptrAddress)
    {
    }

    /// \brief Equality comparison.
    /// \param[in] _other Other RegistrationObjectId object to compare against.
    bool operator==(const RegistrationObjectId &_other) const
    {
      return this->id == _other.id;
    }

    /// \brief Wrapped uintptr_t variable.
    std::uintptr_t id;
  };


  /// \brief A class to hold the queue of event descriptors registered by
  /// translation units. This queue is necessary to ensure that event
  /// creation continues to work after plugins are unloaded. The typical
  /// scenario this aims to solve is:
  ///  1. Plugin P1 registers event descripter for event E1.
  ///  2. Plugin P1 gets unloaded.
  ///  3. Plugin P2 registers a event descriptor for event E1 and tries
  ///     to create an instance of E1.
  /// When P1 gets unloaded, the destructor of the static event
  /// registration object calls EventFactory::Unregister which removes the event
  /// descriptor from the queue. Without this step, P2 would attempt to use
  /// the event descriptor created by P1 in step 3 and likely segfault
  /// because the memory associated with that descriptor has been deleted when
  /// P1 was unloaded.
  class EventDescriptorQueue
  {
    /// \brief Check if the queue is empty
    public: bool GZ_COMMON_HIDDEN Empty()
    {
      return this->queue.empty();
    }

    /// \brief Add an event descriptor to the queue
    /// \param[in] _regObjId An ID that identifies the registration object. This
    /// is generally derived from the `this` pointer of the static event
    /// registration object created when calling GZ_COMMON_REGISTER_EVENT.
    /// \param[in] _eventDesc The event descriptor
    public: void GZ_COMMON_HIDDEN Add(RegistrationObjectId _regObjId,
                     EventDescriptorBase *_eventDesc)
    {
      this->queue.push_front({_regObjId, _eventDesc});
    }

    /// \brief Remove an event descriptor from the queue. This also deletes
    /// memory allocated for the event descriptor by the static event
    /// registration object.
    /// \param[in] _regObjId An ID that identifies the registration object. This
    /// is generally derived from the `this` pointer of the static event
    /// registration object created when calling GZ_COMMON_REGISTER_EVENT.
    public: void GZ_COMMON_HIDDEN Remove(RegistrationObjectId  _regObjId)
    {
      auto eventIt = std::find_if(this->queue.rbegin(), this->queue.rend(),
                                 [&](const auto &_item)
                                 { return _item.first == _regObjId; });

      if (eventIt != this->queue.rend())
      {
        EventDescriptorBase *eventDesc = eventIt->second;
        this->queue.erase(std::prev(eventIt.base()));
        delete eventDesc;
      }
    }

    /// \brief Create an event using the latest available event
    /// descriptor. This simply forward to EventDescriptorBase::Create
    /// \sa EventDescriptorBase::Create
    public: GZ_COMMON_HIDDEN std::unique_ptr<Event> Create() const
    {
      if (!this->queue.empty())
      {
        return this->queue.front().second->Create();
      }
      return {};
    }

    /// \brief Queue of event descriptors registered by static registration
    /// objects.
    private: std::deque<std::pair<RegistrationObjectId,
                                  EventDescriptorBase *>> queue;
  };

  /// \brief A factory that generates an event based on a string type.
  // TODO(azeey) Do not inherit from common::SingletonT in Harmonic
  class EventFactory
      : public gz::common::SingletonT<EventFactory>
  {
    // Deleted copy constructors to make the ABI checker happy
    public: EventFactory(EventFactory &) = delete;
    public: EventFactory(const EventFactory &) = delete;
    // Since the copy constructors are deleted, we need to explicitly declare a
    // default constructor.
    public: EventFactory() = default;

    /// \brief Get an instance of the singleton
    public: GZ_COMMON_VISIBLE static EventFactory *Instance();

    /// \brief Register an event so that the factory can create instances
    /// of the event based on an ID.
    /// \param[in] _type Type of event to register.
    /// \param[in] _eventDesc Object to manage the creation of EventTypeT
    ///  objects.
    /// \param[in] _regObjId An ID that identifies the registration object. This
    /// is generally derived from the `this` pointer of the static event
    /// registration object created when calling GZ_COMMON_REGISTER_EVENT.
    /// \tparam EventTypeT Type of event to register.
    public: template <typename EventTypeT>
    void Register(const char *_type, EventDescriptorBase *_eventDesc,
                  RegistrationObjectId  _regObjId)
    {
      auto typeHash = gz::common::hash64(_type);

      // Initialize static member variable - we need to set these
      // static members for every shared lib that uses the event, but we
      // only add them to the maps below once.
      EventTypeT::typeId = typeHash;
      EventTypeT::typeName = _type;

      // Check if event has already been registered by another library
      auto runtimeName = typeid(EventTypeT).name();
      auto runtimeNameIt = this->runtimeNamesById.find(typeHash);
      if (runtimeNameIt != this->runtimeNamesById.end())
      {
        // Warn user if type was previously registered with a different name.
        // We're leaving the ID set in case this is a false difference across
        // libraries.
        if (runtimeNameIt->second != runtimeName)
        {
          std::cerr
            << "Registered events of different types with same name: type ["
            << runtimeNameIt->second << "] and type [" << runtimeName
            << "] with name [" << _type << "]. Second type will not work."
            << std::endl;
          return;
        }
      }

      // This happens at static initialization time, so we can't use common
      // console
      std::string debugEnv;
      gz::common::env("GZ_DEBUG_EVENT_FACTORY", debugEnv);

      if (debugEnv != "true")
      {
        gz::common::env("IGN_DEBUG_EVENT_FACTORY", debugEnv);
        if (debugEnv == "true")
        {
          std::cerr << "Environment variable [IGN_DEBUG_EVENT_FACTORY] "
                    << "is deprecated! Please use [GZ_DEBUG_EVENT_FACTORY]"
                    << "instead." << std::endl;
        }
      }

      if (debugEnv == "true")
      {
        std::cout << "Registering [" << EventTypeT::typeName << "]"
                  << std::endl;
      }

      // Keep track of all types
      this->eventsById[EventTypeT::typeId].Add(_regObjId, _eventDesc);
      namesById[EventTypeT::typeId] = EventTypeT::typeName;
      runtimeNamesById[EventTypeT::typeId] = runtimeName;
    }

    /// \brief Unregister an event so that the factory can't create instances
    /// of the event anymore.
    /// \tparam EventTypeT Type of event to unregister.
    /// \param[in] _regObjId An ID that identifies the registration object. This
    /// is generally derived from the `this` pointer of the static event
    /// registration object created when calling GZ_COMMON_REGISTER_EVENT.
    public: template<typename EventTypeT>
    void Unregister(RegistrationObjectId  _regObjId)
    {
      this->Unregister(EventTypeT::typeId, _regObjId);
    }

    /// \brief Unregister an event so that the factory can't create instances
    /// of the event anymore.
    /// \details This function will not reset the `typeId` static variable
    /// within the event type itself. Prefer using the templated
    /// `Unregister` function when possible.
    /// \param[in] _typeId Type of event to unregister.
    /// \param[in] _regObjId An ID that identifies the registration object. This
    /// is generally derived from the `this` pointer of the static event
    /// registration object created when calling GZ_COMMON_REGISTER_EVENT.
    public: void Unregister(EventTypeId _typeId,
                            RegistrationObjectId _regObjId)
    {
      auto it = this->eventsById.find(_typeId);
      if (it != this->eventsById.end())
      {
        it->second.Remove(_regObjId);

        if (it->second.Empty())
        {
          this->eventsById.erase(it);
        }
      }
    }

    /// \brief Create a new instance of an event.
    /// \return Pointer to an event. Null if the event
    /// type could not be handled.
    /// \tparam EventTypeT event type requested
    public: template<typename EventTypeT>
    std::unique_ptr<EventTypeT> New()
    {
      return std::unique_ptr<EventTypeT>(static_cast<EventTypeT *>(
            this->New(EventTypeT::typeId).release()));
    }

    /// \brief Create a new instance of an event.
    /// \param[in] _type Event id to create.
    /// \return Pointer to an event. Null if the event
    /// type could not be handled.
    public: std::unique_ptr<Event> New(
        const EventTypeId &_type)
    {
      // Create a new event if a FactoryFn has been assigned to this type.
      std::unique_ptr<Event> event;
      auto it = this->eventsById.find(_type);
      if (it != this->eventsById.end())
      {
        event = it->second.Create();
      }
      return event;
    }

    /// \brief Get all the registered event types by ID.
    /// return Vector of event IDs.
    public: std::vector<EventTypeId> TypeIds() const
    {
      std::vector<EventTypeId> types;

      // Return the list of all known event types.
      for (const auto &event : this->eventsById)
        types.push_back(event.first);

      return types;
    }

    /// \brief Check if an event type has been registered.
    /// return True if registered.
    public: bool HasType(EventTypeId _typeId)
    {
      return this->eventsById.find(_typeId) != this->eventsById.end();
    }

    /// \brief Get an event's type name given its type ID.
    /// return Unique event name.
    public: std::string Name(EventTypeId _typeId) const
    {
      if (this->namesById.find(_typeId) != this->namesById.end())
        return namesById.at(_typeId);

      return "";
    }

    /// \brief A list of registered events where the key is its id.
    private: std::map<EventTypeId, EventDescriptorQueue> eventsById;

    /// \brief A list of IDs and their equivalent names.
    public: std::map<EventTypeId, std::string> namesById;

    /// \brief Keep track of the runtime names for types and warn the user if
    /// they try to register different types with the same typeName.
    public: std::map<EventTypeId, std::string> runtimeNamesById;
  };

  /// \brief Static event registration macro.
  ///
  /// Use this macro to register events.
  ///
  /// \details Each time a plugin which uses an event is loaded, it tries to
  /// register the event again, so we prevent that.
  /// \param[in] _eventType event type name.
  /// \param[in] _classname Class name for event.
  #define GZ_COMMON_REGISTER_EVENT(_eventType, _classname) \
  class GzCommonEvents##_classname \
  { \
    public: GzCommonEvents##_classname() \
    { \
      using namespace gz;\
      using Desc = common::EventDescriptor<_classname>; \
      common::EventFactory::Instance()->Register<_classname>(\
        _eventType, new Desc(), common::RegistrationObjectId(this));\
    } \
    public: GzCommonEvents##_classname( \
                const GzCommonEvents##_classname&) = delete; \
    public: GzCommonEvents##_classname( \
                GzCommonEvents##_classname&) = delete; \
    public: ~GzCommonEvents##_classname() \
    { \
      using namespace gz; \
      common::EventFactory::Instance()->Unregister<_classname>( \
          common::RegistrationObjectId(this)); \
    } \
  }; \
  static GzCommonEvents##_classname\
    GzCommonEventsInitializer##_classname;
// }  // namespace events
}
}  // namespace common
}  // namespace gz

#endif  // GZ_COMMON_EVENTFACTORY_HH_
