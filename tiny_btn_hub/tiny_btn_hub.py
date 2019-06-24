import os
import errno
import logging
import logging.config
import threading
import serial
import json
import time
import queue
from lifxlan import LifxLAN, errors  # , Group
from GroupExt import GroupExt


def setup_logging(
    default_path='logging.json',
    default_level=logging.INFO,
    env_key='LOG_CFG'
):
    """Setup logging configuration"""
    path = default_path
    value = os.getenv(env_key, None)

    try:
        os.makedirs('logs')
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise

    if value:
        path = value

    if os.path.exists(path):
        with open(path, 'rt') as f:
            config = json.load(f)
        logging.config.dictConfig(config)
    else:
        logging.basicConfig(
            level=logging.DEBUG,
            format='[%(levelname)-8s] [%(asctime)s]'
            ' [%(threadName)-12s] %(message)s',
        )


def get_groups():
    groups = {}
    with open('groups.json', 'rt') as f:
        groups = json.load(f)

    return groups


def get_btn_configs():
    button_configs = {}

    with open('btn_configs.json', 'rt') as f:
        button_configs = json.load(f)

    return button_configs


class BtnHub:
    _groups = {}

    def __init__(self, lifxlan, groups, btn_configs, logger=None):
        self.lifx = lifxlan
        self.groups = groups
        self.groups_to_update = queue.Queue()
        self.btn_configs = btn_configs
        self.all_lights = self.lifx.get_lights()
        self.logger = logger or logging.getLogger(__name__)

        self.update_groups()

    def update_groups(self, groups=None):
        if groups is None:
            groups = self.groups

        for group_name, group in groups.items():
            try:
                lights = []
                for light in self.all_lights:
                    if light.get_label() in group["names"]:
                        self.logger.info(
                            "'{}' added to group '{}'"
                            .format(
                                light.get_label(),
                                group_name
                            )
                        )
                        lights.append(light)

                group["devices"] = GroupExt(lights)

                num_devices = len(group["devices"].get_device_list())
                num_names = len(group["names"])

                if num_devices < num_names:
                    # only do this if the initial pass didn't get all_lights
                    # get_devices_by_name uses discovery, which is really slow
                    self.logger.info(
                        "Not all devices were initially added to '{}'. "
                        "Trying Discovery method."
                        .format(group_name)
                    )

                    devices = self.lifx.get_devices_by_name(group["names"])
                    # convert regular Group to GroupExt
                    # should maybe just extend LifxLAN as well
                    group["devices"] = GroupExt(devices.get_device_list())
                    # group["devices"] = devices
                    sep = "', '"
                    self.logger.info(
                        "'{}' added to group '{}'"
                        .format(
                            sep.join(group["names"]),
                            group_name
                        )
                    )
            except errors.WorkflowException:
                self.groups_to_update.put(group_name)
                self.logger.error(
                    "WorkflowException: '{}' group failed to initialize."
                    .format(group_name),
                    exc_info=True
                )


def read_serial(event_queue):
    ser = serial.Serial('/dev/ttyUSB0', 19200, 8, 'N', 1, timeout=1)
    while True:
        output = ser.readline().decode("utf-8").strip()
        if output != '':
            event_queue.put(output)


def update(hub, update_interval=10):
    while True:
        time.sleep(update_interval)
        # get the current queue into a list so
        # that the function works on something static
        groups = list(hub.groups_to_update.queue)
        print(groups)
        if len(groups) > 0:
            hub.logger.info(
                "Updating {} groups. Update interval is {} seconds"
                .format(
                    len(groups),
                    update_interval
                )
            )

            # clear the queue before we update the groups
            # so there are no repeats on the next update
            with hub.groups_to_update.mutex:
                hub.groups_to_update.queue.clear()

            hub.update_groups(groups)


def main():
    lifx = LifxLAN()
    logger = logging.getLogger(__name__)
    groups = get_groups()
    button_configs = get_btn_configs()

    hub = BtnHub(lifx, groups, button_configs, logger)

    event_queue = queue.Queue()
    serial_thread = threading.Thread(
        target=read_serial,
        args=(event_queue,),
        daemon=True
    )
    serial_thread.start()

    update_thread = threading.Thread(
        target=update,
        name="Update",
        args=(hub,),
        daemon=True
    )
    update_thread.start()

    logger.info("Ready")

    while True:
        if event_queue.qsize() > 0:
            message = json.loads(event_queue.get())
            if message["success"]:
                sender = str(message["sender"])
                action = message["action"]
                count = message["count"]

                logger.info(
                    "Sent by: {:2} Action: {:9} Count: {:5}"
                    .format(sender, action, count)
                )

                btn_action = None
                group_name = None

                if (
                    sender in hub.btn_configs
                    and action in hub.btn_configs[sender]
                    and "group" in hub.btn_configs[sender][action]
                    and hub.btn_configs[sender][action]["group"] in hub.groups
                ):
                    btn_action = hub.btn_configs[sender][action]
                    group_name = btn_action["group"]

                if (
                    btn_action is not None
                    and group_name is not None
                ):
                    group = hub.groups[group_name]
                    command = btn_action["command"]
                    devices = group["devices"]

                    cmd = getattr(devices, command, None)
                    if callable(cmd):
                        try:
                            result = ""
                            args_str = ""
                            kwargs_str = ""
                            args_list = []
                            args = ()
                            kwargs = {}

                            if "args" in btn_action:
                                args = tuple(btn_action["args"])
                                args_str = "*args={}".format(str(args))
                                args_list.append(args_str)

                            if "kwargs" in btn_action:
                                kwargs = btn_action["kwargs"]
                                kwargs_str = ', '.join(
                                    [
                                        '{}={!r}'.format(k, v)
                                        for k, v in kwargs.items()
                                    ]
                                )
                                # just gonna say this looks like the
                                # Eye of Sauron or something
                                kwargs_str = "**kwargs={{{0}}}".format(
                                    kwargs_str
                                )
                                args_list.append(kwargs_str)

                            result = cmd(*args, **kwargs)

                            if result is None:
                                result = ""

                            all_args = "({})".format(
                                ','.join(args_list)
                            )

                            logger.info(
                                "group: '{}' "
                                "command: {}{} "
                                "result: {}"
                                .format(
                                    group_name,
                                    command,
                                    all_args,
                                    result
                                )
                            )
                        except errors.WorkflowException:
                            logger.error(
                                "WorkflowException during {} for {}"
                                .format(command, group_name),
                                exc_info=True
                            )
                        except ValueError as e:
                            logger.error(
                                "ValueError: {}".format(e.message),
                                exc_info=True
                            )
                    else:
                        logger.warning(
                            "{} could not be run or is not a function"
                            .format(command)
                        )
                else:
                    logger.warning(
                        "Button or group config not found for this message"
                    )

        time.sleep(0.01)


if __name__ == "__main__":
    setup_logging()
    main()
