import rospy
import asyncio
from sphero_rvr_msgs.srv import SetLeds, SetLedsResponse, \
                                TriggerLedEvent, TriggerLedEventRequest, TriggerLedEventResponse
from sphero_sdk import Colors

class RVRColorPicker:
    def __init__(self, rvr):
        self.rvr = rvr

        self._std_color = [135, 206, 250]
        self._current_color = self._std_color
        self._stop_event_callback = None

        # services
        rospy.Service('set_leds', SetLeds, self.set_leds_callback)
        rospy.Service('trigger_led_event', TriggerLedEvent, self.trigger_led_event_callback)

    def set_leds_callback(self, req):
        self._current_color = [c for c in req.rgb_color]      # convert to list since rospy seems to create tuples ...
        asyncio.run(self.rvr.led_control.set_all_leds_rgb(self._current_color[0], self._current_color[1], self._current_color[2]))

        return SetLedsResponse()

    def trigger_led_event_callback(self, req):
        if req.stop_current_event:
            if self._stop_event_callback is None:
                rospy.logwarn("({}) No event to stop.".format(rospy.get_name()))
                return TriggerLedEventResponse(False)

            asyncio.run(self._stop_event_callback())
            self._stop_event_callback = None
            return TriggerLedEventResponse(True)

        success = asyncio.run(self.trigger_event(req.event_id))
        return TriggerLedEventResponse(success)

    async def trigger_event(self, event_id):
        # activate requested event
        # ---> register new events here
        if event_id == TriggerLedEventRequest.STARTUP:
            await self._blink_event(Colors.green)
            await self._return_to_std_color_event()

        elif event_id == TriggerLedEventRequest.VISION_STARTUP:
            await self._blink_event(Colors.yellow)
            await self._return_to_std_color_event()

        elif event_id == TriggerLedEventRequest.EMERGENCY_STOP:
            await self._blink_event(Colors.red, num_blinks=3)
            await self._return_to_current_color_event()

        elif event_id == TriggerLedEventRequest.START_DRIVING:
            await self._blink_event(Colors.blue, num_blinks=0)
            await self._return_to_current_color_event()

        elif event_id == TriggerLedEventRequest.DOWNLOAD_MODEL:
            await self._download_model_event()

        elif event_id == TriggerLedEventRequest.ERROR:
            await self._blink_event(Colors.red, num_blinks=0)
            await self._return_to_current_color_event()
        else:
            rospy.logwarn("({}) Unknown led event id: {}".format(rospy.get_name(), event_id))
            return False

        return True

    async def _return_to_std_color_event(self):
        self._current_color = self._std_color
        await self._return_to_current_color_event()

    async def _return_to_current_color_event(self):
        await self.rvr.led_control.set_all_leds_rgb(self._current_color[0], self._current_color[1], self._current_color[2])

    async def _download_model_event(self):
        self._stop_event_callback = self._return_to_current_color_event
        await self.rvr.led_control.set_all_leds_color(color=Colors.yellow)

    async def _blink_event(self, color, num_blinks=2):
        # let the rvr blink to show that it is up
        if num_blinks == 0:
            await self.rvr.led_control.set_all_leds_color(color=color)
            await asyncio.sleep(0.3)
        else:
            for i in range(num_blinks):
                await self.rvr.led_control.set_all_leds_color(color=color)
                await asyncio.sleep(0.3)
                await self.rvr.led_control.set_all_leds_color(color=Colors.off)
                await asyncio.sleep(0.3)

