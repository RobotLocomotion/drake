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

// #include "drake/lcm/drake_lcm.h"
// #include "drake/lcm/drake_lcm_base.h"
// #include "drake/lcm/drake_lcm_interface.h"
// #include "drake/lcm/drake_lcm_log.h"
// #include "drake/lcm/drake_lcm_params.h"
// #include "drake/lcm/lcm_messages.h"

// Symbol: pydrake_doc_lcm
constexpr struct /* pydrake_doc_lcm */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::lcm
    struct /* lcm */ {
      // Symbol: drake::lcm::AreLcmMessagesEqual
      struct /* AreLcmMessagesEqual */ {
        // Source: drake/lcm/lcm_messages.h
        const char* doc =
R"""(Compares two LCM messages of the same type to see if they are equal.)""";
      } AreLcmMessagesEqual;
      // Symbol: drake::lcm::DecodeLcmMessage
      struct /* DecodeLcmMessage */ {
        // Source: drake/lcm/lcm_messages.h
        const char* doc =
R"""(Decodes an LCM message from a series of bytes.

Raises:
    RuntimeError if decoding fails.)""";
      } DecodeLcmMessage;
      // Symbol: drake::lcm::DrakeLcm
      struct /* DrakeLcm */ {
        // Source: drake/lcm/drake_lcm.h
        const char* doc =
R"""(A wrapper around a *real* LCM instance.

See allow_network "DRAKE_ALLOW_NETWORK" for an environment variable
option to disable LCM network traffic (i.e., only allowing ``memq://``
URLs).

For network issues on macOS, see
https://drake.mit.edu/troubleshooting.html#lcm-macos.)""";
        // Symbol: drake::lcm::DrakeLcm::DrakeLcm
        struct /* ctor */ {
          // Source: drake/lcm/drake_lcm.h
          const char* doc_0args =
R"""(Constructs using LCM's default URL (either the default hard-coded URL,
or else LCM_DEFAULT_URL environment variable if it is set).)""";
          // Source: drake/lcm/drake_lcm.h
          const char* doc_1args_lcm_url =
R"""(Constructs using the given URL. If empty, it will use the default URL
as per the no-argument constructor.)""";
          // Source: drake/lcm/drake_lcm.h
          const char* doc_1args_params = R"""(Constructs using the given parameters.)""";
        } ctor;
        // Symbol: drake::lcm::DrakeLcm::HandleSubscriptions
        struct /* HandleSubscriptions */ {
          // Source: drake/lcm/drake_lcm.h
          const char* doc = R"""()""";
        } HandleSubscriptions;
        // Symbol: drake::lcm::DrakeLcm::Publish
        struct /* Publish */ {
          // Source: drake/lcm/drake_lcm.h
          const char* doc = R"""()""";
        } Publish;
        // Symbol: drake::lcm::DrakeLcm::Subscribe
        struct /* Subscribe */ {
          // Source: drake/lcm/drake_lcm.h
          const char* doc = R"""()""";
        } Subscribe;
        // Symbol: drake::lcm::DrakeLcm::SubscribeAllChannels
        struct /* SubscribeAllChannels */ {
          // Source: drake/lcm/drake_lcm.h
          const char* doc = R"""()""";
        } SubscribeAllChannels;
        // Symbol: drake::lcm::DrakeLcm::SubscribeMultichannel
        struct /* SubscribeMultichannel */ {
          // Source: drake/lcm/drake_lcm.h
          const char* doc = R"""()""";
        } SubscribeMultichannel;
        // Symbol: drake::lcm::DrakeLcm::available
        struct /* available */ {
          // Source: drake/lcm/drake_lcm.h
          const char* doc =
R"""(Returns true if the LCM runtime library is enabled in this build of
Drake. When false, functions that require the runtime (e.g.,
HandleSubscriptions and Publish) will throw an error. See
``//tools/flags:with_lcm_runtime``.)""";
        } available;
        // Symbol: drake::lcm::DrakeLcm::get_lcm_url
        struct /* get_lcm_url */ {
          // Source: drake/lcm/drake_lcm.h
          const char* doc = R"""()""";
        } get_lcm_url;
      } DrakeLcm;
      // Symbol: drake::lcm::DrakeLcmBase
      struct /* DrakeLcmBase */ {
        // Source: drake/lcm/drake_lcm_base.h
        const char* doc =
R"""(A concrete subclass of DrakeInterface that throws for all functions
except the constructor and destructor. This is useful for subclasses
that only wish to implement a subset of the DrakeLcmInterface
features.)""";
        // Symbol: drake::lcm::DrakeLcmBase::DrakeLcmBase
        struct /* ctor */ {
          // Source: drake/lcm/drake_lcm_base.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::lcm::DrakeLcmBase::HandleSubscriptions
        struct /* HandleSubscriptions */ {
          // Source: drake/lcm/drake_lcm_base.h
          const char* doc = R"""()""";
        } HandleSubscriptions;
        // Symbol: drake::lcm::DrakeLcmBase::Publish
        struct /* Publish */ {
          // Source: drake/lcm/drake_lcm_base.h
          const char* doc = R"""()""";
        } Publish;
        // Symbol: drake::lcm::DrakeLcmBase::Subscribe
        struct /* Subscribe */ {
          // Source: drake/lcm/drake_lcm_base.h
          const char* doc = R"""()""";
        } Subscribe;
        // Symbol: drake::lcm::DrakeLcmBase::SubscribeAllChannels
        struct /* SubscribeAllChannels */ {
          // Source: drake/lcm/drake_lcm_base.h
          const char* doc = R"""()""";
        } SubscribeAllChannels;
        // Symbol: drake::lcm::DrakeLcmBase::SubscribeMultichannel
        struct /* SubscribeMultichannel */ {
          // Source: drake/lcm/drake_lcm_base.h
          const char* doc = R"""()""";
        } SubscribeMultichannel;
        // Symbol: drake::lcm::DrakeLcmBase::get_lcm_url
        struct /* get_lcm_url */ {
          // Source: drake/lcm/drake_lcm_base.h
          const char* doc = R"""()""";
        } get_lcm_url;
      } DrakeLcmBase;
      // Symbol: drake::lcm::DrakeLcmInterface
      struct /* DrakeLcmInterface */ {
        // Source: drake/lcm/drake_lcm_interface.h
        const char* doc =
R"""(A pure virtual interface that enables LCM to be mocked.

Because it must be pure, in general it will receive breaking API
changes without notice. Users should not subclass this interface
directly, but rather use one of the existing subclasses such as
DrakeLcmBase instead.

Similarly, method arguments will receive breaking API changes without
notice. Users should not call this interface directly, but rather use
drake∷lcm∷Publish() or drake∷lcm∷Subscribe() instead.)""";
        // Symbol: drake::lcm::DrakeLcmInterface::DrakeLcmInterface
        struct /* ctor */ {
          // Source: drake/lcm/drake_lcm_interface.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::lcm::DrakeLcmInterface::HandleSubscriptions
        struct /* HandleSubscriptions */ {
          // Source: drake/lcm/drake_lcm_interface.h
          const char* doc =
R"""(Invokes the HandlerFunction callbacks for all subscriptions' pending
messages. If ``timeout_millis`` is >0, blocks for up to that long
until at least one message is handled.

Returns:
    the number of messages handled, or 0 on timeout.

Raises:
    RuntimeError when a subscribed handler throws.)""";
        } HandleSubscriptions;
        // Symbol: drake::lcm::DrakeLcmInterface::HandlerFunction
        struct /* HandlerFunction */ {
          // Source: drake/lcm/drake_lcm_interface.h
          const char* doc =
R"""(A callback used by DrakeLcmInterface∷Subscribe(), with arguments: -
``message_buffer`` A pointer to the byte vector that is the serial
representation of the LCM message. - ``message_size`` The size of
``message_buffer``.

A callback should never throw an exception, because it is indirectly
called from C functions.)""";
        } HandlerFunction;
        // Symbol: drake::lcm::DrakeLcmInterface::MultichannelHandlerFunction
        struct /* MultichannelHandlerFunction */ {
          // Source: drake/lcm/drake_lcm_interface.h
          const char* doc =
R"""(A callback used by DrakeLcmInterface∷SubscribeMultipleChannels (which
therefore needs the receiving channel passed in).)""";
        } MultichannelHandlerFunction;
        // Symbol: drake::lcm::DrakeLcmInterface::Publish
        struct /* Publish */ {
          // Source: drake/lcm/drake_lcm_interface.h
          const char* doc =
R"""(Most users should use the drake∷lcm∷Publish() free function, instead
of this interface method.

Publishes an LCM message on channel ``channel``.

Parameter ``channel``:
    The channel on which to publish the message. Must not be the empty
    string.

Parameter ``data``:
    A buffer containing the serialized bytes of the message to
    publish.

Parameter ``data_size``:
    The length of @data in bytes.

Parameter ``time_sec``:
    Time in seconds when the publish event occurred. If unknown, use
    nullopt or a default-constructed optional.)""";
        } Publish;
        // Symbol: drake::lcm::DrakeLcmInterface::Subscribe
        struct /* Subscribe */ {
          // Source: drake/lcm/drake_lcm_interface.h
          const char* doc =
R"""(Most users should use the drake∷lcm∷Subscribe() free function or the
drake∷lcm∷Subscriber wrapper class, instead of this interface method.

Subscribes to an LCM channel without automatic message decoding. The
handler will be invoked when a message arrives on channel ``channel``.

The handler should never throw an exception, because it is indirectly
called from C functions.

Parameter ``channel``:
    The channel to subscribe to. Must not be the empty string. To use
    a regex, see SubscribeMultichannel().

Returns:
    the object used to manage the subscription if that is supported,
    or else nullptr if not supported. The unsubscribe-on-delete
    default is ``False``. Refer to the DrakeSubscriptionInterface
    class overview for details.)""";
        } Subscribe;
        // Symbol: drake::lcm::DrakeLcmInterface::SubscribeAllChannels
        struct /* SubscribeAllChannels */ {
          // Source: drake/lcm/drake_lcm_interface.h
          const char* doc =
R"""(Subscribe to all channels; this is useful for logging and redirecting
LCM traffic without regard to its content.)""";
        } SubscribeAllChannels;
        // Symbol: drake::lcm::DrakeLcmInterface::SubscribeMultichannel
        struct /* SubscribeMultichannel */ {
          // Source: drake/lcm/drake_lcm_interface.h
          const char* doc =
R"""(Subscribes to all channels whose name matches the given regular
expression. The ``regex`` is treated as an anchored "match" not a
"search", i.e., it must match the entire channel name. The specific
regular expression grammar is left unspecified, so it's best to use
only patterns that have identical semantics in all grammars, e.g.,
``".*"``.)""";
        } SubscribeMultichannel;
        // Symbol: drake::lcm::DrakeLcmInterface::get_lcm_url
        struct /* get_lcm_url */ {
          // Source: drake/lcm/drake_lcm_interface.h
          const char* doc =
R"""(Returns a URL describing the transport of this LCM interface.

When the URL refers to a transport offered by LCM itself (e.g., memq
or udpm), then this function must return the conventional URL
spelling. If the implementation of DrakeLcmInterface is using a
non-standard back end, the result implementation-defined.

In either case, it is always formatted using URI syntax rules per the
RFC(s).)""";
        } get_lcm_url;
      } DrakeLcmInterface;
      // Symbol: drake::lcm::DrakeLcmLog
      struct /* DrakeLcmLog */ {
        // Source: drake/lcm/drake_lcm_log.h
        const char* doc =
R"""(A LCM interface for logging LCM messages to a file or playing back
from a existing log. Note the user is responsible for offsetting the
clock used to generate the log and the clock used for playback. For
example, if the log is generated by some external logger (the
lcm-logger binary), which uses the unix epoch time clock to record
message arrival time, the user needs to offset those timestamps
properly to match and the clock used for playback.)""";
        // Symbol: drake::lcm::DrakeLcmLog::DispatchMessageAndAdvanceLog
        struct /* DispatchMessageAndAdvanceLog */ {
          // Source: drake/lcm/drake_lcm_log.h
          const char* doc =
R"""(Let ``MSG`` be the next message event in the log, if ``current_time``
matches ``MSG`'s timestamp, for every DrakeLcmMessageHandlerInterface
`sub`` that's subscribed to ``MSG`'s channel, invoke `sub`'s
HandleMessage method. Then, this function advances the log by exactly
one message. This function does nothing if `MSG`` is null (end of log)
or ``current_time`` does not match `MSG`'s timestamp.

Raises:
    RuntimeError if this instance is not constructed in read-only
    mode.)""";
        } DispatchMessageAndAdvanceLog;
        // Symbol: drake::lcm::DrakeLcmLog::DrakeLcmLog
        struct /* ctor */ {
          // Source: drake/lcm/drake_lcm_log.h
          const char* doc =
R"""(Constructs a DrakeLcmLog.

Parameter ``file_name``:
    Log's file name for reading or writing.

Parameter ``is_write``:
    If false, this instance reads from the Lcm log identified by
    ``file_name``. If true, this instance writes to the Lcm log whose
    name is given by ``file_name``.

Parameter ``overwrite_publish_time_with_system_clock``:
    This parameter only affects the Publish method in write-only mode.
    If true, override the ``second`` parameter passed to Publish
    method, and use host system's clock to generate the timestamp for
    the logged message. This is used to mimic lcm-logger's behavior.
    It also implicitly records how fast the messages are generated in
    real time.

Raises:
    RuntimeError if unable to open file.)""";
        } ctor;
        // Symbol: drake::lcm::DrakeLcmLog::GetNextMessageTime
        struct /* GetNextMessageTime */ {
          // Source: drake/lcm/drake_lcm_log.h
          const char* doc =
R"""(Returns the time in seconds for the next logged message's occurrence
time or infinity if there are no more messages in the current log.

Raises:
    RuntimeError if this instance is not constructed in read-only
    mode.)""";
        } GetNextMessageTime;
        // Symbol: drake::lcm::DrakeLcmLog::HandleSubscriptions
        struct /* HandleSubscriptions */ {
          // Source: drake/lcm/drake_lcm_log.h
          const char* doc =
R"""(This is a no-op for Read mode, and an exception in Write mode.)""";
        } HandleSubscriptions;
        // Symbol: drake::lcm::DrakeLcmLog::Publish
        struct /* Publish */ {
          // Source: drake/lcm/drake_lcm_log.h
          const char* doc =
R"""(Writes an entry occurred at ``timestamp`` with content ``data`` to the
log file. The current implementation blocks until writing is done.

Parameter ``channel``:
    Channel name.

Parameter ``data``:
    Pointer to raw bytes.

Parameter ``data_size``:
    Number of bytes in ``data``.

Parameter ``time_sec``:
    Time in seconds when the message is published. Since messages are
    save to the log file in the order of Publish calls, this function
    should only be called with non-decreasing ``second``. Note that
    this parameter can be overwritten by the host system's clock if
    ``overwrite_publish_time_with_system_clock`` is true at
    construction time.

Raises:
    RuntimeError if this instance is not constructed in write-only
    mode.)""";
        } Publish;
        // Symbol: drake::lcm::DrakeLcmLog::Subscribe
        struct /* Subscribe */ {
          // Source: drake/lcm/drake_lcm_log.h
          const char* doc =
R"""(Subscribes ``handler`` to ``channel``. Multiple handlers can subscribe
to the same channel.

Raises:
    RuntimeError if this instance is not constructed in read-only
    mode.

Returns:
    nullptr because this implementation does not support unsubscribe.)""";
        } Subscribe;
        // Symbol: drake::lcm::DrakeLcmLog::SubscribeAllChannels
        struct /* SubscribeAllChannels */ {
          // Source: drake/lcm/drake_lcm_log.h
          const char* doc =
R"""(Subscribe to all channels; this is useful for logging and redirecting
LCM traffic without regard to its content.

Raises:
    RuntimeError if this instance is not constructed in read-only
    mode.

Returns:
    nullptr because this implementation does not support unsubscribe.)""";
        } SubscribeAllChannels;
        // Symbol: drake::lcm::DrakeLcmLog::SubscribeMultichannel
        struct /* SubscribeMultichannel */ {
          // Source: drake/lcm/drake_lcm_log.h
          const char* doc =
R"""(This function is not yet supported for LCM logs, and will always
throw.)""";
        } SubscribeMultichannel;
        // Symbol: drake::lcm::DrakeLcmLog::available
        struct /* available */ {
          // Source: drake/lcm/drake_lcm_log.h
          const char* doc =
R"""(Returns true if the LCM runtime library is enabled in this build of
Drake. When false, functions that require the runtime (which at the
moment is all functions, including the constructor) will throw an
error. See ``//tools/flags:with_lcm_runtime``.)""";
        } available;
        // Symbol: drake::lcm::DrakeLcmLog::get_lcm_url
        struct /* get_lcm_url */ {
          // Source: drake/lcm/drake_lcm_log.h
          const char* doc = R"""()""";
        } get_lcm_url;
        // Symbol: drake::lcm::DrakeLcmLog::is_write
        struct /* is_write */ {
          // Source: drake/lcm/drake_lcm_log.h
          const char* doc =
R"""(Returns true if this instance is constructed in write-only mode.)""";
        } is_write;
        // Symbol: drake::lcm::DrakeLcmLog::second_to_timestamp
        struct /* second_to_timestamp */ {
          // Source: drake/lcm/drake_lcm_log.h
          const char* doc =
R"""(Converts time (in seconds) relative to the starting time passed to the
constructor to a timestamp in microseconds.)""";
        } second_to_timestamp;
        // Symbol: drake::lcm::DrakeLcmLog::timestamp_to_second
        struct /* timestamp_to_second */ {
          // Source: drake/lcm/drake_lcm_log.h
          const char* doc =
R"""(Converts ``timestamp`` (in microseconds) to time (in seconds) relative
to the starting time passed to the constructor.)""";
        } timestamp_to_second;
      } DrakeLcmLog;
      // Symbol: drake::lcm::DrakeLcmParams
      struct /* DrakeLcmParams */ {
        // Source: drake/lcm/drake_lcm_params.h
        const char* doc =
R"""(The set of parameters for configuring DrakeLcm.)""";
        // Symbol: drake::lcm::DrakeLcmParams::Serialize
        struct /* Serialize */ {
          // Source: drake/lcm/drake_lcm_params.h
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
        } Serialize;
        // Symbol: drake::lcm::DrakeLcmParams::channel_suffix
        struct /* channel_suffix */ {
          // Source: drake/lcm/drake_lcm_params.h
          const char* doc =
R"""(The custom LCM channel name suffix for this DrakeLcm instance
(optional).

When provided, calls to DrakeLcm∷Publish() or DrakeLcm∷Subscribe()
will append this string to the ``channel`` name requested for publish
or subscribe.

For example, with the channel_suffix set to "_ALT" a call to
``Publish(&drake_lcm, "FOO", message)`` will transmit on the network
using the channel name "FOO_ALT", and a call to ``Subscribe(&lcm,
"BAR", handler)`` will only call the handler for messages received on
the "BAR_ALT" channel name.

Simiarly, DrakeLcm∷SubscribeMultichannel() and
DrakeLcm∷SubscribeAllChannels() only subscribe to network messages
that end with the suffix. A network message on a non-matching channel
name (e.g., "QUUX") will silently discarded. The
DrakeLcmInterface∷MultichannelHandlerFunction callback will be passed
the *unadaorned* channel name as its first argument (e.g., "FOO" or
"BAR"), not "FOO_ALT", etc.)""";
        } channel_suffix;
        // Symbol: drake::lcm::DrakeLcmParams::defer_initialization
        struct /* defer_initialization */ {
          // Source: drake/lcm/drake_lcm_params.h
          const char* doc =
R"""((Advanced) Controls whether or not LCM's background receive thread
will be launched immediately during the constructor (when false) or
deferred until the first time it's needed (when true). This can be
useful if the scheduling configuration for new threads varies between
the construction time and first use.)""";
        } defer_initialization;
        // Symbol: drake::lcm::DrakeLcmParams::lcm_url
        struct /* lcm_url */ {
          // Source: drake/lcm/drake_lcm_params.h
          const char* doc =
R"""(The URL for DrakeLcm communication. If empty, DrakeLcm will use the
default URL per the DrakeLcm∷DrakeLcm() no-argument constructor.)""";
        } lcm_url;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("channel_suffix", channel_suffix.doc),
            std::make_pair("defer_initialization", defer_initialization.doc),
            std::make_pair("lcm_url", lcm_url.doc),
          };
        }
      } DrakeLcmParams;
      // Symbol: drake::lcm::DrakeSubscriptionInterface
      struct /* DrakeSubscriptionInterface */ {
        // Source: drake/lcm/drake_lcm_interface.h
        const char* doc =
R"""(A helper class returned by DrakeLcmInterface∷Subscribe() that allows
for (possibly automatic) unsubscription and/or queue capacity control.
Refer to that method for additional details.

Instance of this object are always stored in ``std∷shared_ptr`` to
manage them as resources. When a particular DrakeLcmInterface
implementation does not support subscription controls, the managed
pointer will be ``nullptr`` instead of an instance of this object.

To unsubscribe, induce a call to the DrakeSubscriptionInterface
destructor by bringing the ``std∷shared_ptr`` use count to zero. That
usually means either a call to ``subscription.reset()`` or by allowing
it to go out of scope.

To *disable* unsubscription so that the pointer loss *never* causes
unsubscription, call
``subscription->set_unsubscribe_on_delete(false)``. To *enable*
unsubscription, set it to ``True``. Which choice is active by default
is specified by whatever method returns this object.)""";
        // Symbol: drake::lcm::DrakeSubscriptionInterface::DrakeSubscriptionInterface
        struct /* ctor */ {
          // Source: drake/lcm/drake_lcm_interface.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::lcm::DrakeSubscriptionInterface::set_queue_capacity
        struct /* set_queue_capacity */ {
          // Source: drake/lcm/drake_lcm_interface.h
          const char* doc =
R"""(Sets this subscription's queue depth to store messages between calls
to DrakeLcmInterface∷HandleSubscriptions. When the queue becomes full,
new received messages will be discarded. The default depth is 1.

Warning:
    The memq:// LCM URL does not support per-channel queues, so this
    method has no effect when memq is being used, e.g., in Drake unit
    tests.)""";
        } set_queue_capacity;
        // Symbol: drake::lcm::DrakeSubscriptionInterface::set_unsubscribe_on_delete
        struct /* set_unsubscribe_on_delete */ {
          // Source: drake/lcm/drake_lcm_interface.h
          const char* doc =
R"""(Sets whether or not the subscription on DrakeLcmInterface will be
terminated when this object is deleted. It is permitted to call this
method many times, with a new ``enabled`` value each time.)""";
        } set_unsubscribe_on_delete;
      } DrakeSubscriptionInterface;
      // Symbol: drake::lcm::EncodeLcmMessage
      struct /* EncodeLcmMessage */ {
        // Source: drake/lcm/lcm_messages.h
        const char* doc =
R"""(Encodes an LCM message to a series of bytes.)""";
      } EncodeLcmMessage;
      // Symbol: drake::lcm::LcmHandleSubscriptionsUntil
      struct /* LcmHandleSubscriptionsUntil */ {
        // Source: drake/lcm/drake_lcm_interface.h
        const char* doc =
R"""(Convenience function that repeatedly calls
``lcm->HandleSubscriptions()`` with a timeout value of
``timeout_millis``, until ``finished()`` returns true. Returns the
total number of messages handled.)""";
      } LcmHandleSubscriptionsUntil;
      // Symbol: drake::lcm::Publish
      struct /* Publish */ {
        // Source: drake/lcm/drake_lcm_interface.h
        const char* doc =
R"""(Publishes an LCM message on channel ``channel``.

Parameter ``lcm``:
    The LCM service on which to publish the message. Must not be null.

Parameter ``channel``:
    The channel on which to publish the message. Must not be the empty
    string.

Parameter ``message``:
    The message to publish.

Parameter ``time_sec``:
    Time in seconds when the publish event occurred. If unknown, use
    the default value of nullopt.)""";
      } Publish;
      // Symbol: drake::lcm::Subscribe
      struct /* Subscribe */ {
        // Source: drake/lcm/drake_lcm_interface.h
        const char* doc =
R"""(Subscribes to an LCM channel named ``channel`` and decodes messages of
type ``Message``. See also drake∷lcm∷Subscriber for a simple way to
passively observe received messages, without the need to write a
handler function.

Parameter ``lcm``:
    The LCM service on which to subscribe. Must not be null.

Parameter ``channel``:
    The channel on which to subscribe. Must not be the empty string.

Parameter ``handler``:
    The callback when a message is received and decoded without error.

Parameter ``on_error``:
    The callback when a message is received and cannot be decoded; if
    no error handler is given, an exception is thrown instead.

Returns:
    the object used to unsubscribe if that is supported, or else
    nullptr if unsubscribe is not supported. The unsubscribe-on-delete
    default is ``False``, so that ignoring this result leaves the
    subscription intact. Refer to the DrakeSubscriptionInterface class
    overview for details.

Note:
    Depending on the specific DrakeLcmInterface implementation, the
    handler might be invoked on a different thread than this function.)""";
      } Subscribe;
      // Symbol: drake::lcm::Subscriber
      struct /* Subscriber */ {
        // Source: drake/lcm/drake_lcm_interface.h
        const char* doc =
R"""(Subscribes to and stores a copy of the most recent message on a given
channel, for some ``Message`` type. All copies of a given Subscriber
share the same underlying data. This class does NOT provide any mutex
behavior for multi-threaded locking; it should only be used in cases
where the governing DrakeLcmInterface∷HandleSubscriptions is called
from the same thread that owns all copies of this object.)""";
        // Symbol: drake::lcm::Subscriber::Subscriber<Message>
        struct /* ctor */ {
          // Source: drake/lcm/drake_lcm_interface.h
          const char* doc =
R"""(Subscribes to the (non-empty) ``channel`` on the given (non-null)
``lcm`` instance. The ``lcm`` pointer is only used during
construction; it is not retained by this object. When a undecodable
message is received, ``on_error`` handler is invoked; when
``on_error`` is not provided, an exception will be thrown instead.)""";
        } ctor;
        // Symbol: drake::lcm::Subscriber::clear
        struct /* clear */ {
          // Source: drake/lcm/drake_lcm_interface.h
          const char* doc =
R"""(Clears all data (sets the message and count to all zeros).)""";
        } clear;
        // Symbol: drake::lcm::Subscriber::count
        struct /* count */ {
          // Source: drake/lcm/drake_lcm_interface.h
          const char* doc =
R"""(Returns the total number of received messages.)""";
        } count;
        // Symbol: drake::lcm::Subscriber::message
        struct /* message */ {
          // Source: drake/lcm/drake_lcm_interface.h
          const char* doc =
R"""(Returns the most recently received message, or a value-initialized
(zeros) message otherwise.)""";
        } message;
      } Subscriber;
    } lcm;
  } drake;
} pydrake_doc_lcm;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
