import copy
import datetime
import optparse
import random
import time
from casas import objects, rabbitmq
import logging
from sensor_processor import process_data  # so that we can pass the data off

'''
This is 99.9% from Brian. 
'''


def my_callback(obj_list):
    for obj in obj_list:
        print(str(obj))
    return


class TestAgent(object):
    def __init__(self, options):
        self.agent_name = options.agent_name
        log_level = logging.INFO
        if options.debug:
            log_level = logging.DEBUG
        log_file = options.log_file
        print_log = options.printout
        self.logger = logging.getLogger()
        self.logger.setLevel(log_level)
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        if log_file != None:
            fh = logging.handlers.TimedRotatingFileHandler(log_file,
                                                           when='midnight',
                                                           backupCount=7)
            fh.setLevel(log_level)
            fh.setFormatter(formatter)
            self.logger.addHandler(fh)
        if print_log:
            ch = logging.StreamHandler()
            ch.setLevel(log_level)
            ch.setFormatter(formatter)
            self.logger.addHandler(ch)

        self.translate = dict()

        # You can also set the amqp_vhost="/tokyo" to listen to the events
        # from the EME206 lab.
        self.rcon = rabbitmq.Connection(agent_name=self.agent_name,
                                        amqp_user="ras",
                                        amqp_pass="RASpass42",
                                        amqp_host="smarthomedata.io",
                                        amqp_port="5671",
                                        amqp_vhost="/tokyo",
                                        amqp_ssl=True,
                                        translations=self.translate)
        return

    def add_exchange_1(self):
        self.rcon.setup_subscribe_to_exchange(exchange_name="all.events.testbed.casas",
                                              exchange_type='topic',
                                              routing_key='#',
                                              exchange_durable=True,
                                              casas_events=True,
                                              callback_function=self.callback_1)
        return

    def callback_1(self, obj_list):
        for obj in obj_list:
            # print(obj)
            # print(type(obj.stamp))
            # print(type(obj.target))
            process_data(obj.target, obj.message, obj.sensor_type)
        return

    def run(self):
        try:
            self.rcon.run()
        except KeyboardInterrupt:
            self.stop()
        return

    def stop(self):
        self.rcon.stop()
        return


if __name__ == "__main__":
    parser = optparse.OptionParser(usage="usage: %prog [options]")
    parser.add_option("--agent-name",
                      dest="agent_name",
                      help="Name of the agent (ex. ZigbeeAgent).",
                      default="agent-name")
    parser.add_option("--log-file",
                      dest="log_file",
                      help="Filename if you want to write the log to disk.")
    parser.add_option("--debug",
                      dest="debug",
                      action="store_true",
                      help="Set logging level to DEBUG from INFO.",
                      default=False)
    parser.add_option("--printout",
                      dest="printout",
                      action="store_true",
                      help="Print output to the screen at given logging level.",
                      default=False)
    (options, args) = parser.parse_args()

    agent = TestAgent(options)
    agent.add_exchange_1()
    agent.run()
