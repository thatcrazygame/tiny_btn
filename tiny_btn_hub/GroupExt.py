from lifxlan import Group
from threading import Thread

MAX_HSBK = 65535


class GroupExt(Group):

    # Toggles a group of lights on or off.
    # By default, if all lights are on they will turn off and vice versa
    # If not all lights are the same state initially the group will flip
    # to the opposite of the current majority.
    # If there is a tie default_to defines the behavior.
    def toggle(self, default_to="off", duration=0, rapid=False):
        num_lights_on = 0
        num_lights_off = 0
        for d in self.devices:
            if d.get_power() == MAX_HSBK:
                num_lights_on += 1
            else:
                num_lights_off += 1

        power = "off"
        if num_lights_on == num_lights_off:
            power = default_to
        elif num_lights_on > num_lights_off:
            power = "off"
        else:
            power = "on"

        self.set_power(power, duration, rapid)

        return power

    def on(self, duration=0, rapid=False):
        self.set_power("on", duration, rapid)

    def off(self, duration=0, rapid=False):
        self.set_power("off", duration, rapid)

    def get_average_brightness(self):
        pass

    # positive values of increment increase the brightness
    # negative values decrease it
    # wrap=True causes the brightness to cycle through the brightness
    # levels continuously. Otherwise, it stops increasing or
    # decreasing at the max and min values
    def increment_brightness(
        self,
        increment,
        duration=0,
        rapid=False,
        wrap=True
    ):
        color_supporting_devices = []
        for d in self.devices:
            if d.supports_color:
                color_supporting_devices.append(d)
        # get colors
        colors = []
        for d in color_supporting_devices:
            colors.append(d.get_color())
        # "simultaneous" change
        threads = []
        new_brightnesses = []
        for (i, d) in enumerate(color_supporting_devices):
            hue, saturation, brightness, kelvin = colors[i]
            brightness = brightness + increment
            # Calculating 1% because we don't want the light
            # to be "on" but at 0% brightness
            one_percent = int(round(MAX_HSBK / 100))

            if brightness < one_percent:
                if wrap:
                    brightness = MAX_HSBK
                    # longer transition since it will go from
                    # very dim to max brightness
                    if duration < 2000:
                        duration = 2000
                else:
                    brightness = one_percent

            if brightness > MAX_HSBK:
                if wrap:
                    brightness = one_percent
                    # longer transition since it will go from
                    # very bright to almost off
                    if duration < 2000:
                        duration = 2000
                else:
                    brightness = MAX_HSBK

            new_brightnesses.append(brightness)

            color = [hue, saturation, brightness, kelvin]
            t = Thread(target=d.set_color, args=(color, duration, rapid))
            threads.append(t)
            t.start()
        for t in threads:
            t.join()

        return new_brightnesses

    def increment_brightness_by_percent(
        self,
        increment_pct,  # -99 to 99
        duration=0,
        rapid=False,
        wrap=True
    ):
        if not (-99 <= increment_pct <= 99):
            raise ValueError("increment_pct must be between -99 and 99")

        increment = int(round(MAX_HSBK * increment_pct / 100))
        raw_brightnesses = self.increment_brightness(
            increment,
            duration,
            rapid,
            wrap
        )

        new_brightnesses = [
            "{}%".format(int(round(brightness / MAX_HSBK * 100)))
            for brightness in raw_brightnesses
        ]

        # for brightness in raw_brightnesses:
        #     brightness = "{}%".format()

        return new_brightnesses
