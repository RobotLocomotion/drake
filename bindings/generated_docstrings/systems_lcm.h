#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#include <array>
#include <utility>

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/systems/lcm/lcm_buses.h"
// #include "drake/systems/lcm/lcm_config_functions.h"
// #include "drake/systems/lcm/lcm_interface_system.h"
// #include "drake/systems/lcm/lcm_log_playback_system.h"
// #include "drake/systems/lcm/lcm_publisher_system.h"
// #include "drake/systems/lcm/lcm_scope_system.h"
// #include "drake/systems/lcm/lcm_subscriber_system.h"
// #include "drake/systems/lcm/lcm_system_graphviz.h"
// #include "drake/systems/lcm/serializer.h"

// Symbol: pydrake_doc_systems_lcm
constexpr struct /* pydrake_doc_systems_lcm */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::lcm
    struct /* lcm */ {
    } lcm;
    // Symbol: drake::systems
    struct /* systems */ {
      // Symbol: drake::systems::lcm
      struct /* lcm */ {
        // Symbol: drake::systems::lcm::ApplyLcmBusConfig
        struct /* ApplyLcmBusConfig */ {
          // Source: drake/systems/lcm/lcm_config_functions.h
          const char* doc =
R"""(Given LCM bus names and (nullable) parameters, adds an
LcmInterfaceSystem within the given diagram builder for each bus, and
returns an LcmBuses object that provides access to the
drake∷lcm∷DrakeLcmInterface objects that were created.

Because the interfaces live within the builder (and so eventually, the
diagram), the diagram will pump the interfaces when it's used with a
simulator. Refer to the LcmInterfaceSystem documentation for details.

The interface pointers remain owned by the builder; the LcmBuses
object merely aliases into the builder (and then eventually, the
diagram).

As a special case, the user can opt-out of LCM either by passing
``nullopt`` as the drake∷lcm∷DrakeLcmParams, or by setting the URL
within the DrakeLcmParams to LcmBuses∷kLcmUrlMemqNull. In that case,
only a drake∷lcm∷DrakeLcmInterface object will be created (not a full
LcmInterfaceSystem), and the LCM messages will *not* be pumped.

Parameter ``lcm_buses``:
    A map of {bus_name: params} for LCM transceivers, to be used by
    drivers, sensors, etc.)""";
        } ApplyLcmBusConfig;
        // Symbol: drake::systems::lcm::FindOrCreateLcmBus
        struct /* FindOrCreateLcmBus */ {
          // Source: drake/systems/lcm/lcm_config_functions.h
          const char* doc =
R"""((Advanced) Returns an LCM interface based on a convenient set of
heuristics.

If the ``forced_result`` is non-null, then returns ``forced_result``
and does nothing else.

Otherwise, if ``lcm_buses`` is null and ``bus_name`` is "default",
then creates a new DrakeLcm object owned by the ``builder`` and
returns a pointer to it.

Otherwise, if ``lcm_buses`` is null, then throws an exception.

Otherwise, returns the ``lcm_buses->Find(description_of_caller,
bus_name)`` which might throw if there is so such ``bus_name``.

The return value is an alias into memory owned elsewhere (typically by
a DiagramBuilder or a Diagram) and is never nullptr.

Parameter ``forced_result``:
    can be null

Parameter ``lcm_buses``:
    can be null

Parameter ``builder``:
    must not be null)""";
        } FindOrCreateLcmBus;
        // Symbol: drake::systems::lcm::LcmBuses
        struct /* LcmBuses */ {
          // Source: drake/systems/lcm/lcm_buses.h
          const char* doc =
R"""(A mapping from {bus_name: interface} with sugar for error checking
with nice error messages during access.

Note that this class is shallow-const. A user of a const LcmBuses
object cannot add or remove buses, but can retrieve a non-const
DrakeLcmInterface pointer and then "modify" the object it points to by
subscribing to a channel.)""";
          // Symbol: drake::systems::lcm::LcmBuses::Add
          struct /* Add */ {
            // Source: drake/systems/lcm/lcm_buses.h
            const char* doc =
R"""(Adds a bus. Throws if the bus is nullptr, or if there was already a
bus of the same name.)""";
          } Add;
          // Symbol: drake::systems::lcm::LcmBuses::Find
          struct /* Find */ {
            // Source: drake/systems/lcm/lcm_buses.h
            const char* doc =
R"""(Finds the bus of the given name, or throws if there is no such bus.

The return value is an alias into memory owned elsewhere (typically by
a DiagramBuilder or a Diagram) and is never nullptr.

Parameter ``description_of_caller``:
    is a noun phrase that will be used when creating an error message.
    A typical value would be something like "Camera 5", "Robot
    controller", "The default visualizer", or etc.)""";
          } Find;
          // Symbol: drake::systems::lcm::LcmBuses::GetAllBusNames
          struct /* GetAllBusNames */ {
            // Source: drake/systems/lcm/lcm_buses.h
            const char* doc =
R"""(Returns a list of all known bus_name keys.)""";
          } GetAllBusNames;
          // Symbol: drake::systems::lcm::LcmBuses::LcmBuses
          struct /* ctor */ {
            // Source: drake/systems/lcm/lcm_buses.h
            const char* doc = R"""(Constructs an empty mapping.)""";
          } ctor;
          // Symbol: drake::systems::lcm::LcmBuses::size
          struct /* size */ {
            // Source: drake/systems/lcm/lcm_buses.h
            const char* doc = R"""(Returns the total number of buses.)""";
          } size;
        } LcmBuses;
        // Symbol: drake::systems::lcm::LcmInterfaceSystem
        struct /* LcmInterfaceSystem */ {
          // Source: drake/systems/lcm/lcm_interface_system.h
          const char* doc =
R"""(LcmInterfaceSystem acts within a Diagram to allow LcmSubscriberSystem
instances to receive data from the network during a simulation. When
its parent Diagram is run via the Simulator, the LcmSubscriberSystem
sources will output new values based on new data received over LCM.

This System has no inputs nor outputs nor state nor parameters; it
declares only an update event that pumps LCM messages into their
subscribers iff the LCM stack has message(s) waiting. The subscribers
will then update their outputs using their own declared events.

Note that because this class implements DrakeLcmInterface, any
subscriber registered on that interface will be serviced during
simulation, not just `LcmSubscriberSystem`s.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {cpp}
    DiagramBuilder<double> builder;
    auto lcm = builder.AddSystem<LcmInterfaceSystem>();
    auto subscriber = builder.AddSystem(
        LcmSubscriberSystem∷Make<lcmt_drake_signal>("channel", lcm));

.. raw:: html

    </details>)""";
          // Symbol: drake::systems::lcm::LcmInterfaceSystem::HandleSubscriptions
          struct /* HandleSubscriptions */ {
            // Source: drake/systems/lcm/lcm_interface_system.h
            const char* doc = R"""()""";
          } HandleSubscriptions;
          // Symbol: drake::systems::lcm::LcmInterfaceSystem::LcmInterfaceSystem
          struct /* ctor */ {
            // Source: drake/systems/lcm/lcm_interface_system.h
            const char* doc_1args_lcm_url =
R"""(Constructs using the given URL. With no URL, uses the LCM_DEFAULT_URL
environment variable iff it is set or else the default hard-coded URL.)""";
            // Source: drake/systems/lcm/lcm_interface_system.h
            const char* doc_1args_params = R"""(Constructs using the given params.)""";
            // Source: drake/systems/lcm/lcm_interface_system.h
            const char* doc_1args_lcm =
R"""(Constructs using the given LCM service. The pointer is aliased by this
class and must remain valid for the lifetime of this object. Users
MUST NOT start the receive thread on this object.)""";
          } ctor;
          // Symbol: drake::systems::lcm::LcmInterfaceSystem::Publish
          struct /* Publish */ {
            // Source: drake/systems/lcm/lcm_interface_system.h
            const char* doc = R"""()""";
          } Publish;
          // Symbol: drake::systems::lcm::LcmInterfaceSystem::Subscribe
          struct /* Subscribe */ {
            // Source: drake/systems/lcm/lcm_interface_system.h
            const char* doc = R"""()""";
          } Subscribe;
          // Symbol: drake::systems::lcm::LcmInterfaceSystem::SubscribeAllChannels
          struct /* SubscribeAllChannels */ {
            // Source: drake/systems/lcm/lcm_interface_system.h
            const char* doc = R"""()""";
          } SubscribeAllChannels;
          // Symbol: drake::systems::lcm::LcmInterfaceSystem::SubscribeMultichannel
          struct /* SubscribeMultichannel */ {
            // Source: drake/systems/lcm/lcm_interface_system.h
            const char* doc = R"""()""";
          } SubscribeMultichannel;
          // Symbol: drake::systems::lcm::LcmInterfaceSystem::get_lcm_url
          struct /* get_lcm_url */ {
            // Source: drake/systems/lcm/lcm_interface_system.h
            const char* doc = R"""()""";
          } get_lcm_url;
        } LcmInterfaceSystem;
        // Symbol: drake::systems::lcm::LcmLogPlaybackSystem
        struct /* LcmLogPlaybackSystem */ {
          // Source: drake/systems/lcm/lcm_log_playback_system.h
          const char* doc =
R"""(Advances the cursor of a drake∷lcm∷DrakeLcmLog based on the timestamps
seen the Context that is used to simulate this System.

This is useful when a simulated Diagram contains
LcmSubscriberSystem(s) whose outputs should be determined by logged
data and when the log's cursor should advance automatically during
simulation.)""";
          // Symbol: drake::systems::lcm::LcmLogPlaybackSystem::DoCalcNextUpdateTime
          struct /* DoCalcNextUpdateTime */ {
            // Source: drake/systems/lcm/lcm_log_playback_system.h
            const char* doc = R"""()""";
          } DoCalcNextUpdateTime;
          // Symbol: drake::systems::lcm::LcmLogPlaybackSystem::LcmLogPlaybackSystem
          struct /* ctor */ {
            // Source: drake/systems/lcm/lcm_log_playback_system.h
            const char* doc =
R"""(Constructs a playback system that advances the given ``log``.

Parameter ``log``:
    non-null pointer that is aliased and retained by this object.)""";
          } ctor;
        } LcmLogPlaybackSystem;
        // Symbol: drake::systems::lcm::LcmPublisherSystem
        struct /* LcmPublisherSystem */ {
          // Source: drake/systems/lcm/lcm_publisher_system.h
          const char* doc =
R"""(Publishes an LCM message containing information from its input port.
Optionally sends a one-time initialization message. Publishing can be
set up to happen on a per-step or periodic basis. Publishing "by
force", through ``LcmPublisherSystem∷Publish(const Context&)``, is
also enabled.

Note:
    You should generally provide an LCM interface yourself, since
    there should normally be just one of these typically-heavyweight
    objects per program. However, if you're sure there isn't any other
    need for an LCM interface in your program, you can let
    LcmPublisherSystem allocate and maintain a drake∷lcm∷DrakeLcm
    object internally.

.. pydrake_system::

    name: LcmPublisherSystem
    input_ports:
    - lcm_message)""";
          // Symbol: drake::systems::lcm::LcmPublisherSystem::AddInitializationMessage
          struct /* AddInitializationMessage */ {
            // Source: drake/systems/lcm/lcm_publisher_system.h
            const char* doc =
R"""(Specifies a message-publishing function to be invoked once from an
initialization event. If this method is not called, no initialization
event will be created.

You can only call this method once.

Raises:
    RuntimeError if called a second time.

Precondition:
    The publisher function may not be null.)""";
          } AddInitializationMessage;
          // Symbol: drake::systems::lcm::LcmPublisherSystem::InitializationPublisher
          struct /* InitializationPublisher */ {
            // Source: drake/systems/lcm/lcm_publisher_system.h
            const char* doc =
R"""(This is the type of an initialization message publisher that can be
provided via AddInitializationMessage().)""";
          } InitializationPublisher;
          // Symbol: drake::systems::lcm::LcmPublisherSystem::LcmPublisherSystem
          struct /* ctor */ {
            // Source: drake/systems/lcm/lcm_publisher_system.h
            const char* doc_5args =
R"""(A constructor for an LcmPublisherSystem that takes LCM message objects
on its sole abstract-valued input port. The LCM message type is
determined by the provided ``serializer``. Will publish on forced
events and either periodic or per-step events, as determined by
publish_period.

Parameter ``channel``:
    The LCM channel on which to publish.

Parameter ``serializer``:
    The serializer that converts between byte vectors and LCM message
    objects. Cannot be null.

Parameter ``lcm``:
    A pointer to the LCM subsystem to use, which must remain valid for
    the lifetime of this object. If null, a drake∷lcm∷DrakeLcm object
    is allocated and maintained internally, but see the note in the
    class comments.

Parameter ``publish_period``:
    Period that messages will be published (optional). If the publish
    period is zero, LcmPublisherSystem will use per-step publishing
    instead; see LeafSystem∷DeclarePerStepPublishEvent().

Parameter ``publish_offset``:
    Offset that messages will be published (optional), in seconds.
    This adds a phase-shift delay to periodic publish events.

Precondition:
    publish_period is non-negative.

Precondition:
    publish_offset is finite and non-negative.

Precondition:
    publish_offset is set to zero when the publish_period is zero.)""";
            // Source: drake/systems/lcm/lcm_publisher_system.h
            const char* doc_6args =
R"""(A constructor for an LcmPublisherSystem that takes LCM message objects
on its sole abstract-valued input port. The LCM message type is
determined by the provided ``serializer``.

Parameter ``channel``:
    The LCM channel on which to publish.

Parameter ``serializer``:
    The serializer that converts between byte vectors and LCM message
    objects. Cannot be null.

Parameter ``lcm``:
    A pointer to the LCM subsystem to use, which must remain valid for
    the lifetime of this object. If null, a drake∷lcm∷DrakeLcm object
    is allocated and maintained internally, but see the note in the
    class comments.

Parameter ``publish_triggers``:
    Set of triggers that determine when messages will be published.
    Supported TriggerTypes are {kForced, kPeriodic, kPerStep}. Will
    throw an exception if empty or if unsupported types are provided.

Parameter ``publish_period``:
    Period that messages will be published (optional). publish_period
    should only be non-zero if one of the publish_triggers is
    kPerStep.

Parameter ``publish_offset``:
    Offset that messages will be published (optional), in seconds.
    This adds a phase-shift delay to periodic publish events.

Precondition:
    publish_triggers contains a subset of {kForced, kPeriodic,
    kPerStep}.

Precondition:
    publish_period is non-negative.

Precondition:
    publish_period > 0 if and only if publish_triggers contains
    kPeriodic.

Precondition:
    publish_offset is finite and non-negative.

Precondition:
    publish_offset > 0 if and only if publish_triggers contains
    kPeriodic.)""";
          } ctor;
          // Symbol: drake::systems::lcm::LcmPublisherSystem::Make
          struct /* Make */ {
            // Source: drake/systems/lcm/lcm_publisher_system.h
            const char* doc_4args =
R"""(A factory method that returns an LcmPublisherSystem that takes
Value<LcmMessage> message objects on its sole abstract-valued input
port.

Sets the default set of publish triggers: if publish_period = 0,
publishes on forced events and per step, if publish_period > 0,
publishes on forced events and periodically.

Template parameter ``LcmMessage``:
    message type to serialize, e.g., lcmt_drake_signal.

Parameter ``channel``:
    The LCM channel on which to publish.

Parameter ``lcm``:
    A pointer to the LCM subsystem to use, which must remain valid for
    the lifetime of this object. If null, a drake∷lcm∷DrakeLcm object
    is allocated and maintained internally, but see the note in the
    class comments.

Parameter ``publish_period``:
    Period that messages will be published (optional). If the publish
    period is zero, LcmPublisherSystem will use per-step publishing
    instead; see LeafSystem∷DeclarePerStepPublishEvent().

Parameter ``publish_offset``:
    Offset that messages will be published (optional), in seconds.
    This adds a phase-shift delay to periodic publish events.

Precondition:
    publish_period is non-negative.

Precondition:
    publish_offset is finite and non-negative.

Precondition:
    publish_offset is set to zero when the publish_period is zero.)""";
            // Source: drake/systems/lcm/lcm_publisher_system.h
            const char* doc_5args =
R"""(A factory method for an LcmPublisherSystem that takes LCM message
objects on its sole abstract-valued input port. The LCM message type
is determined by the provided ``serializer``.

Parameter ``channel``:
    The LCM channel on which to publish.

Parameter ``lcm``:
    A pointer to the LCM subsystem to use, which must remain valid for
    the lifetime of this object. If null, a drake∷lcm∷DrakeLcm object
    is allocated and maintained internally, but see the note in the
    class comments.

Parameter ``publish_triggers``:
    Set of triggers that determine when messages will be published.
    Supported TriggerTypes are {kForced, kPeriodic, kPerStep}. Will
    throw an error if empty or if unsupported types are provided.

Parameter ``publish_period``:
    Period that messages will be published (optional). publish_period
    should only be non-zero if one of the publish_triggers is
    kPeriodic.

Parameter ``publish_offset``:
    Offset that messages will be published (optional), in seconds.
    This adds a phase-shift delay to periodic publish events.

Precondition:
    publish_triggers contains a subset of {kForced, kPeriodic,
    kPerStep}.

Precondition:
    publish_period is non-negative.

Precondition:
    publish_period > 0 if and only if publish_triggers contains
    kPeriodic.

Precondition:
    publish_offset is finite and non-negative.

Precondition:
    publish_offset > 0 if and only if publish_triggers contains
    kPeriodic.)""";
          } Make;
          // Symbol: drake::systems::lcm::LcmPublisherSystem::get_channel_name
          struct /* get_channel_name */ {
            // Source: drake/systems/lcm/lcm_publisher_system.h
            const char* doc =
R"""(Returns the channel name supplied during construction.)""";
          } get_channel_name;
          // Symbol: drake::systems::lcm::LcmPublisherSystem::get_publish_offset
          struct /* get_publish_offset */ {
            // Source: drake/systems/lcm/lcm_publisher_system.h
            const char* doc =
R"""(Returns the publish_offset provided at construction time.)""";
          } get_publish_offset;
          // Symbol: drake::systems::lcm::LcmPublisherSystem::get_publish_period
          struct /* get_publish_period */ {
            // Source: drake/systems/lcm/lcm_publisher_system.h
            const char* doc =
R"""(Returns the publish_period provided at construction time.)""";
          } get_publish_period;
          // Symbol: drake::systems::lcm::LcmPublisherSystem::lcm
          struct /* lcm */ {
            // Source: drake/systems/lcm/lcm_publisher_system.h
            const char* doc =
R"""(Returns a mutable reference to the LCM object in use by this
publisher. This may have been supplied in the constructor or may be an
internally-maintained object of type drake∷lcm∷DrakeLcm.)""";
          } lcm;
          // Symbol: drake::systems::lcm::LcmPublisherSystem::make_name
          struct /* make_name */ {
            // Source: drake/systems/lcm/lcm_publisher_system.h
            const char* doc =
R"""(Returns the default name for a system that publishes ``channel``.)""";
          } make_name;
        } LcmPublisherSystem;
        // Symbol: drake::systems::lcm::LcmScopeSystem
        struct /* LcmScopeSystem */ {
          // Source: drake/systems/lcm/lcm_scope_system.h
          const char* doc =
R"""(LcmScopeSystem provides the ability to convert any vector output port
to a simple LCM message and publish that message periodically. The
intention is to instrument complex diagrams using external tools like
``lcm-spy``.

.. pydrake_system::

    name: LcmScopeSystem
    input_ports:
    - input
    output_ports:
    - output)""";
          // Symbol: drake::systems::lcm::LcmScopeSystem::AddToBuilder
          struct /* AddToBuilder */ {
            // Source: drake/systems/lcm/lcm_scope_system.h
            const char* doc =
R"""(Adds an LcmScopeSystem and LcmPublisherSystem to the given
``builder``.

Parameter ``lcm``:
    A pointer to the LCM subsystem to use, which must remain valid for
    the lifetime of ``builder``. This will typically be an instance of
    an LcmInterfaceSystem that's already been added to the
    ``builder``.

Parameter ``signal``:
    The output port to be scoped. Must be an OutputPort of a system
    that's already been added to the ``builder``.

Parameter ``channel``:
    The LCM channel on which to publish.

Parameter ``publish_period``:
    Specifies how often messages will be published. If the period is
    zero, the LcmPublisherSystem will publish every step.)""";
          } AddToBuilder;
          // Symbol: drake::systems::lcm::LcmScopeSystem::LcmScopeSystem
          struct /* ctor */ {
            // Source: drake/systems/lcm/lcm_scope_system.h
            const char* doc =
R"""((Advanced.) Most users will use AddToBuilder instead of this
constructor. On its own, this constructor does not publish anything.

Creates a system with one input port and one output port. The input
accepts a vector of the given ``size``. The output produces an
lcmt_scope message.)""";
          } ctor;
        } LcmScopeSystem;
        // Symbol: drake::systems::lcm::LcmSubscriberSystem
        struct /* LcmSubscriberSystem */ {
          // Source: drake/systems/lcm/lcm_subscriber_system.h
          const char* doc =
R"""(Receives LCM messages from a given channel and outputs them to a
System<double>'s port. This class stores the most recently processed
LCM message in the State. When a LCM message arrives asynchronously,
an update event is scheduled to process the message and store it in
the State at the earliest possible simulation time. The output is
always consistent with the State.

To process a LCM message, CalcNextUpdateTime() needs to be called
first to check for new messages and schedule a callback event if a new
LCM message has arrived. The message is then processed and stored in
the Context by CalcUnrestrictedUpdate(). When this system is evaluated
by the Simulator, all these operations are taken care of by the
Simulator. On the other hand, the user needs to manually replicate
this process without the Simulator.

If LCM service in use is a drake∷lcm∷DrakeLcmLog (not live operation),
then see drake∷systems∷lcm∷LcmLogPlaybackSystem for a helper to
advance the log cursor in concert with the simulation.

.. pydrake_system::

    name: LcmSubscriberSystem
    output_ports:
    - y0)""";
          // Symbol: drake::systems::lcm::LcmSubscriberSystem::GetInternalMessageCount
          struct /* GetInternalMessageCount */ {
            // Source: drake/systems/lcm/lcm_subscriber_system.h
            const char* doc =
R"""(Returns the internal message counter. Meant to be used with
``WaitForMessage``.)""";
          } GetInternalMessageCount;
          // Symbol: drake::systems::lcm::LcmSubscriberSystem::GetMessageCount
          struct /* GetMessageCount */ {
            // Source: drake/systems/lcm/lcm_subscriber_system.h
            const char* doc =
R"""(Returns the message counter stored in ``context``.)""";
          } GetMessageCount;
          // Symbol: drake::systems::lcm::LcmSubscriberSystem::LcmSubscriberSystem
          struct /* ctor */ {
            // Source: drake/systems/lcm/lcm_subscriber_system.h
            const char* doc =
R"""(Constructor that returns a subscriber System that provides message
objects on its sole abstract-valued output port. The type of the
message object is determined by the ``serializer``.

Parameter ``channel``:
    The LCM channel on which to subscribe.

Parameter ``serializer``:
    The serializer that converts between byte vectors and LCM message
    objects. Cannot be null.

Parameter ``lcm``:
    A non-null pointer to the LCM subsystem to subscribe on. If
    ``wait_for_message_on_initialization_timeout > 0``, then the
    pointer must remain valid for the lifetime of the returned system.

Parameter ``wait_for_message_on_initialization_timeout``:
    Configures the behavior of initialization events (see
    System∷ExecuteInitializationEvents() and Simulator∷Initialize())
    by specifying the number of seconds (wall-clock elapsed time) to
    wait for a new message. If this timeout is <= 0, initialization
    will copy any already-received messages into the Context but will
    not process any new messages. If this timeout is > 0,
    initialization will call lcm->HandleSubscriptions() until at least
    one message is received or until the timeout. Pass ∞ to wait
    indefinitely.)""";
          } ctor;
          // Symbol: drake::systems::lcm::LcmSubscriberSystem::Make
          struct /* Make */ {
            // Source: drake/systems/lcm/lcm_subscriber_system.h
            const char* doc =
R"""(Factory method that returns a subscriber System that provides
Value<LcmMessage> message objects on its sole abstract-valued output
port.

Template parameter ``LcmMessage``:
    message type to deserialize, e.g., lcmt_drake_signal.

Parameter ``channel``:
    The LCM channel on which to subscribe.

Parameter ``lcm``:
    A non-null pointer to the LCM subsystem to subscribe on. If
    ``wait_for_message_on_initialization_timeout > 0``, then the
    pointer must remain valid for the lifetime of the returned system.

Parameter ``wait_for_message_on_initialization_timeout``:
    Configures the behavior of initialization events (see
    System∷ExecuteInitializationEvents() and Simulator∷Initialize())
    by specifying the number of seconds (wall-clock elapsed time) to
    wait for a new message. If this timeout is <= 0, initialization
    will copy any already-received messages into the Context but will
    not process any new messages. If this timeout is > 0,
    initialization will call lcm->HandleSubscriptions() until at least
    one message is received or until the timeout. Pass ∞ to wait
    indefinitely.)""";
          } Make;
          // Symbol: drake::systems::lcm::LcmSubscriberSystem::WaitForMessage
          struct /* WaitForMessage */ {
            // Source: drake/systems/lcm/lcm_subscriber_system.h
            const char* doc =
R"""(Blocks the caller until its internal message count exceeds
``old_message_count`` with an optional timeout.

Parameter ``old_message_count``:
    Internal message counter.

Parameter ``message``:
    If non-null, will return the received message.

Parameter ``timeout``:
    The duration (in seconds) to wait before returning; a non-positive
    duration will not time out.

Returns:
    Returns the new count of received messages. If a timeout occurred,
    this will be less than or equal to old_message_count.

Precondition:
    If ``message`` is specified, this system must be abstract-valued.)""";
          } WaitForMessage;
          // Symbol: drake::systems::lcm::LcmSubscriberSystem::get_channel_name
          struct /* get_channel_name */ {
            // Source: drake/systems/lcm/lcm_subscriber_system.h
            const char* doc = R"""()""";
          } get_channel_name;
          // Symbol: drake::systems::lcm::LcmSubscriberSystem::make_name
          struct /* make_name */ {
            // Source: drake/systems/lcm/lcm_subscriber_system.h
            const char* doc =
R"""(Returns the default name for a system that subscribes to ``channel``.)""";
          } make_name;
        } LcmSubscriberSystem;
        // Symbol: drake::systems::lcm::Serializer
        struct /* Serializer */ {
          // Source: drake/systems/lcm/serializer.h
          const char* doc =
R"""(Serializer is specific to a single LcmMessage type, and translates
between LCM message bytes and drake∷Value<LcmMessage> objects.

Template parameter ``LcmMessage``:
    message type to serialize, e.g., lcmt_drake_signal.)""";
          // Symbol: drake::systems::lcm::Serializer::CreateDefaultValue
          struct /* CreateDefaultValue */ {
            // Source: drake/systems/lcm/serializer.h
            const char* doc = R"""()""";
          } CreateDefaultValue;
          // Symbol: drake::systems::lcm::Serializer::Deserialize
          struct /* Deserialize */ {
            // Source: drake/systems/lcm/serializer.h
            const char* doc = R"""()""";
          } Deserialize;
          // Symbol: drake::systems::lcm::Serializer::Serialize
          struct /* Serialize */ {
            // Source: drake/systems/lcm/serializer.h
            const char* doc = R"""()""";
          } Serialize;
          // Symbol: drake::systems::lcm::Serializer::Serializer<LcmMessage>
          struct /* ctor */ {
            // Source: drake/systems/lcm/serializer.h
            const char* doc = R"""()""";
          } ctor;
        } Serializer;
        // Symbol: drake::systems::lcm::SerializerInterface
        struct /* SerializerInterface */ {
          // Source: drake/systems/lcm/serializer.h
          const char* doc =
R"""(SerializerInterface translates between LCM message bytes and
drake∷AbstractValue objects that contain LCM messages, e.g., a
Value<lcmt_drake_signal>. All ``const`` methods are threadsafe. See
Serializer for a message-specific concrete subclass.)""";
          // Symbol: drake::systems::lcm::SerializerInterface::CreateDefaultValue
          struct /* CreateDefaultValue */ {
            // Source: drake/systems/lcm/serializer.h
            const char* doc =
R"""(Creates a value-initialized (zeroed) instance of the message object.
The result can be used as the output object filled in by Deserialize.)""";
          } CreateDefaultValue;
          // Symbol: drake::systems::lcm::SerializerInterface::Deserialize
          struct /* Deserialize */ {
            // Source: drake/systems/lcm/serializer.h
            const char* doc =
R"""(Translates LCM message bytes into a drake∷AbstractValue object.)""";
          } Deserialize;
          // Symbol: drake::systems::lcm::SerializerInterface::Serialize
          struct /* Serialize */ {
            // Source: drake/systems/lcm/serializer.h
            const char* doc =
R"""(Translates a drake∷AbstractValue object into LCM message bytes.)""";
          } Serialize;
          // Symbol: drake::systems::lcm::SerializerInterface::SerializerInterface
          struct /* ctor */ {
            // Source: drake/systems/lcm/serializer.h
            const char* doc = R"""()""";
          } ctor;
        } SerializerInterface;
      } lcm;
    } systems;
  } drake;
} pydrake_doc_systems_lcm;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
