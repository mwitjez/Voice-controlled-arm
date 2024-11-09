import logging
import ask_sdk_core.utils as ask_utils
import os
import json
import time
import boto3
from ask_sdk_core.skill_builder import SkillBuilder
from ask_sdk_core.dispatch_components import AbstractRequestHandler
from ask_sdk_core.dispatch_components import AbstractExceptionHandler
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model import Response
from awscrt import io, mqtt, auth, http
from awsiot import mqtt_connection_builder
import sys
import threading
from uuid import uuid4
from boto3.dynamodb.conditions import Key, Attr
from botocore.exceptions import ClientError

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

endpoint = ""
port = 8883
root_ca = ""
key = ""
cert = ""
client_id = "lambda-" + str(uuid4())
topic = "voice"


def format_mqtt_message(directive):
    payload = {}
    payload['message'] = directive

    print("Payload")
    print(json.dumps(payload))

    return json.dumps(payload)


def format_complex_mqtt_message_base(directive, direction, degrees):
    payload = {}
    payload['message'] = directive
    payload['direction'] = direction
    payload['degrees'] = degrees

    print("Payload")
    print(json.dumps(payload))

    return json.dumps(payload)


def format_complex_mqtt_message_arm(directive, direction, value):
    payload = {}
    payload['message'] = directive
    payload['direction'] = direction
    payload['value'] = value

    print("Payload")
    print(json.dumps(payload))

    return json.dumps(payload)


def send_mqtt_directive(topic,
                        directive,
                        direction=None,
                        degrees=None,
                        value=None):
    payload = format_mqtt_message(directive)
    try:
        if directive == "move_base":
            payload = format_complex_mqtt_message_base(directive, direction,
                                                       degrees)
        elif directive == "move_arm":
            payload = format_complex_mqtt_message_arm(directive, direction,
                                                      value)
        else:
            payload = format_mqtt_message(directive)
        mqtt_connection.publish(topic=topic,
                                payload=payload,
                                qos=mqtt.QoS.AT_LEAST_ONCE)
        print("SEND")
    except Exception as e:
        print("An exception occurred")
        print(e)
    time.sleep(0.1)
    # Disconnect
    """ print("Disconnecting...")
    disconnect_future = mqtt_connection.disconnect()
    disconnect_future.result()
    print("Disconnected!") """


def on_connection_interrupted(connection, error, **kwargs):
    print("Connection interrupted. error: {}".format(error))


# Callback when an interrupted connection is re-established.


def on_connection_resumed(connection, return_code, session_present, **kwargs):
    print("Connection resumed. return_code: {} session_present: {}".format(
        return_code, session_present))

    if return_code == mqtt.ConnectReturnCode.ACCEPTED and not session_present:
        print("Session did not persist. Resubscribing to existing topics...")
        resubscribe_future, _ = connection.resubscribe_existing_topics()

        # Cannot synchronously wait for resubscribe result because we're on the connection's event-loop thread,
        # evaluate result with a callback instead.
        resubscribe_future.add_done_callback(on_resubscribe_complete)


def on_resubscribe_complete(resubscribe_future):
    resubscribe_results = resubscribe_future.result()
    print("Resubscribe results: {}".format(resubscribe_results))

    for topic, qos in resubscribe_results['topics']:
        if qos is None:
            sys.exit("Server rejected resubscribe to topic: {}".format(topic))


class LaunchRequestHandler(AbstractRequestHandler):
    """Handler for Skill Launch."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool

        return ask_utils.is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speak_output = "We have a city to burn!"

        return (handler_input.response_builder.speak(speak_output).ask(
            speak_output).response)


class OpenIntentHandler(AbstractRequestHandler):
    """Handler for Open Intent."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_intent_name("OpenIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        send_mqtt_directive("voice", "open")
        time.sleep(0.5)

        speak_output = "Opening!"

        return (
            handler_input.response_builder.speak(
                speak_output).set_should_end_session(False)
            # .ask("add a reprompt if you want to keep the session open for the user to respond")
            .response)


class CloseIntentHandler(AbstractRequestHandler):
    """Handler for Close Intent."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_intent_name("CloseIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        send_mqtt_directive("voice", "close")
        time.sleep(0.5)

        speak_output = "Closing!"

        return (
            handler_input.response_builder.speak(
                speak_output).set_should_end_session(False)
            # .ask("add a reprompt if you want to keep the session open for the user to respond")
            .response)


class StopIntentHandler(AbstractRequestHandler):
    """Handler for Stop Intent."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_intent_name("StopIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        send_mqtt_directive("voice", "stop")
        time.sleep(0.5)

        speak_output = "Stoping!"

        return (
            handler_input.response_builder.speak(
                speak_output).set_should_end_session(False)
            # .ask("add a reprompt if you want to keep the session open for the user to respond")
            .response)


class PickUpIntentHandler(AbstractRequestHandler):
    """Handler for PickUp Intent."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_intent_name("PickUpIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        send_mqtt_directive("voice", "pick")
        time.sleep(0.5)

        speak_output = "Picking up!"

        return (
            handler_input.response_builder.speak(
                speak_output).set_should_end_session(False)
            # .ask("add a reprompt if you want to keep the session open for the user to respond")
            .response)


class MoveArmIntent(AbstractRequestHandler):
    """Handler for MoveArmIntent Intent."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_intent_name("MoveArmIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        slots = handler_input.request_envelope.request.intent.slots
        direction = slots["arm_direction"]
        value = slots["value"]
        send_mqtt_directive("voice",
                            "move_arm",
                            direction.value,
                            value=value.value)
        time.sleep(0.5)

        speak_output = "Moving " + direction.value + " " + value.value + " centimeters."

        return (
            handler_input.response_builder.speak(
                speak_output).set_should_end_session(False)
            # .ask("add a reprompt if you want to keep the session open for the user to respond")
            .response)


class MoveBaseIntent(AbstractRequestHandler):
    """Handler for MoveBaseIntent Intent."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_intent_name("MoveBaseIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        slots = handler_input.request_envelope.request.intent.slots
        direction = slots["direction"]
        degrees = slots["degrees"]
        send_mqtt_directive("voice",
                            "move_base",
                            direction.value,
                            degrees=degrees.value)
        time.sleep(0.5)

        speak_output = "Moving " + direction.value + " " + degrees.value + " degrees."

        return (
            handler_input.response_builder.speak(
                speak_output).set_should_end_session(False)
            # .ask("add a reprompt if you want to keep the session open for the user to respond")
            .response)


class HelpIntentHandler(AbstractRequestHandler):
    """Handler for Help Intent."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_intent_name("AMAZON.HelpIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speak_output = "You can give commands to robot like open gripper or close gripper. How can I help?"

        return (handler_input.response_builder.speak(speak_output).ask(
            speak_output).response)


class CancelOrStopIntentHandler(AbstractRequestHandler):
    """Single handler for Cancel and Stop Intent."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return (ask_utils.is_intent_name("AMAZON.CancelIntent")(handler_input)
                or
                ask_utils.is_intent_name("AMAZON.StopIntent")(handler_input))

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speak_output = "Goodbye!"

        return (handler_input.response_builder.speak(speak_output).response)


class FallbackIntentHandler(AbstractRequestHandler):
    """Single handler for Fallback Intent."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_intent_name("AMAZON.FallbackIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        logger.info("In FallbackIntentHandler")
        speech = "Hmm, I'm not sure. You can give commands to robot like open gripper. What would you like to do?"
        reprompt = "I didn't catch that. What can I help you with?"

        return handler_input.response_builder.speak(speech).ask(
            reprompt).response


class SessionEndedRequestHandler(AbstractRequestHandler):
    """Handler for Session End."""
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_request_type("SessionEndedRequest")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response

        # Any cleanup logic goes here.

        return handler_input.response_builder.response


class IntentReflectorHandler(AbstractRequestHandler):
    """The intent reflector is used for interaction model testing and debugging.
    It will simply repeat the intent the user said. You can create custom handlers
    for your intents by defining them above, then also adding them to the request
    handler chain below.
    """
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return ask_utils.is_request_type("IntentRequest")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        intent_name = ask_utils.get_intent_name(handler_input)
        speak_output = "You just triggered " + intent_name + "."

        return (
            handler_input.response_builder.speak(speak_output)
            # .ask("add a reprompt if you want to keep the session open for the user to respond")
            .response)


class CatchAllExceptionHandler(AbstractExceptionHandler):
    """Generic error handling to capture any syntax or routing errors. If you receive an error
    stating the request handler chain is not found, you have not implemented a handler for
    the intent being invoked or included it in the skill builder below.
    """
    def can_handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> bool
        return True

    def handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> Response
        logger.error(exception, exc_info=True)

        speak_output = "Sorry, I had trouble doing what you asked. Please try again."

        return (handler_input.response_builder.speak(speak_output).ask(
            speak_output).response)


# The SkillBuilder object acts as the entry point for your skill, routing all request and response
# payloads to the handlers above. Make sure any new handlers or interceptors you've
# defined are included below. The order matters - they're processed top to bottom.
event_loop_group = io.EventLoopGroup(1)
host_resolver = io.DefaultHostResolver(event_loop_group)
client_bootstrap = io.ClientBootstrap(event_loop_group, host_resolver)

proxy_options = None

mqtt_connection = mqtt_connection_builder.mtls_from_path(
    endpoint=endpoint,
    port=port,
    cert_filepath=cert,
    pri_key_filepath=key,
    client_bootstrap=client_bootstrap,
    ca_filepath=root_ca,
    on_connection_interrupted=on_connection_interrupted,
    on_connection_resumed=on_connection_resumed,
    client_id=client_id,
    clean_session=False,
    keep_alive_secs=30,
    http_proxy_options=proxy_options)

connect_future = mqtt_connection.connect()

# Future.result() waits until a result is available
connect_future.result()
print("Connected!")

sb = SkillBuilder()

sb.add_request_handler(LaunchRequestHandler())
sb.add_request_handler(OpenIntentHandler())
sb.add_request_handler(CloseIntentHandler())
sb.add_request_handler(StopIntentHandler())
sb.add_request_handler(PickUpIntentHandler())
sb.add_request_handler(MoveArmIntent())
sb.add_request_handler(MoveBaseIntent())
sb.add_request_handler(HelpIntentHandler())
sb.add_request_handler(CancelOrStopIntentHandler())
sb.add_request_handler(FallbackIntentHandler())
sb.add_request_handler(SessionEndedRequestHandler())
# make sure IntentReflectorHandler is last so it doesn't override your custom intent handlers
sb.add_request_handler(IntentReflectorHandler())

sb.add_exception_handler(CatchAllExceptionHandler())

lambda_handler = sb.lambda_handler()
