#*****************************************************************************#
#**
#**  CASAS RabbitMQ Python Library
#**
#**    Brian L Thomas, 2017
#**
#** Tools by the Center for Advanced Studies in Adaptive Systems at
#**  the School of Electrical Engineering and Computer Science at
#**  Washington State University
#** 
#** Copyright Washington State University, 2017
#** Copyright Brian L. Thomas, 2017
#** 
#** All rights reserved
#** Modification, distribution, and sale of this work is prohibited without
#**  permission from Washington State University
#** 
#** If this code is used for public research, any resulting publications need
#** to cite work done by Brian L. Thomas at the Center for Advanced Study of 
#** Adaptive Systems (CASAS) at Washington State University.
#** 
#** Contact: Brian L. Thomas (brian.thomas@email.wsu.edu)
#** Contact: Diane J. Cook (cook@eecs.wsu.edu)
#*****************************************************************************#
import copy
import datetime
import json
import logging
import logging.handlers
import pika
import pytz
import re
import threading
import time
import uuid

from .objects import Event, Tag, Control, build_objects_from_json



class ConsumeCallback(object):
    
    def __init__(self, casas_events=True, callback_function=None,
                 is_exchange=False, exchange_name=None, is_queue=False,
                 queue_name=None, limit_to_sensor_types=list(), no_ack=False,
                 translations=dict(), timezone=dict()):
        self.l = logging.getLogger(__name__).getChild('ConsumeCallback')
        self.casas_events = casas_events
        self.callback_function = callback_function
        self.is_exchange = is_exchange
        self.exchange_name = exchange_name
        self.is_queue = is_queue
        self.queue_name = queue_name
        self.limit_to_sensor_types = copy.deepcopy(limit_to_sensor_types)
        self.is_limiting = False
        if len(self.limit_to_sensor_types) > 0:
            self.is_limiting = True
        self.no_ack = no_ack
        self.translations = translations
        self.timezone = timezone
        return
    
    def on_message(self, channel, basic_deliver, properties, body):
        """
        Invoked by pika when a message is delivered from RabbitMQ. The
        channel is passed for your convenience. The basic_deliver object that
        is passed in carries the exchange, routing key, delivery tag and
        a redelivered flag for the message. The properties passed in is an
        instance of BasicProperties with the message properties and the body
        is the message that was sent.

        :param pika.channel.Channel channel: The channel object
        :param pika.Spec.Basic.Deliver: basic_deliver method
        :param pika.Spec.BasicProperties: properties
        :param str|unicode body: The message body

        """
        self.l.debug("on_message({})".format(str(body)))
        
        if self.casas_events:
            obj = build_objects_from_json(body)
            self.l.debug("obj = {}".format(str(obj)))
            if self.is_limiting:
                remove_list = list()
                for i in range(len(obj)):
                    if obj[i].sensor_type not in self.limit_to_sensor_types:
                        remove_list.append(i)
                remove_list.reverse()
                for i in remove_list:
                    del obj[i]
            self.l.debug("size of obj = {}, size of limit = {}".format(str(len(obj)),str(len(self.limit_to_sensor_types))))
            if len(obj) > 0:
                for e in obj:
                    if e.site in self.translations:
                        if e.target in self.translations[e.site]:
                            e.sensor_1,e.sensor_2 = self.translations[e.site][e.target]
                    if e.site in self.timezone:
                        e.stamp_local = pytz.utc.localize(e.stamp, is_dst=None).astimezone(self.timezone[e.site])
                    else:
                        e.stamp_local = pytz.utc.localize(e.stamp, is_dst=None).astimezone(self.timezone['default'])
                    e.stamp_local = e.stamp_local.replace(tzinfo=None)
                self.callback_function(obj)
        else:
            self.callback_function(body)
        
        if not self.no_ack:
            channel.basic_ack(delivery_tag=basic_deliver.delivery_tag)
        return



class Connection:
    """
    This is an example consumer that will handle unexpected interactions
    with RabbitMQ such as channel and connection closures.

    If RabbitMQ closes the connection, it will reopen it. You should
    look at the output, as there are limited reasons why the connection may
    be closed, which usually are tied to permission related issues or
    socket timeouts.

    If the channel is closed, it will indicate a problem with one of the
    commands that were issued and that should surface in the output as well.
    """

    def __init__(self, agent_name, amqp_user, amqp_pass, amqp_host, amqp_port,
                 amqp_vhost='/', amqp_ssl=True, translations=None,
                 timezone=None):
        """
        Create a new instance of the CASAS RammitMQ Connection class.

        :param str agent_name: The name of the agent using the RabbitMQ
                   connection, used in logging and debugging
        :param str amqp_user: The RabbitMQ username
        :param str amqp_pass: The RabbitMQ password
        :param str amqp_host: The RabbitMQ hostname
        :param str amqp_port: The RabbitMQ port to use for connecting
        :param str amqp_vhost: (optional) The RabbitMQ virtual host to connect
                   to, with a default value of '/'
        :param bool amqp_ssl: (optional) Defines if using SSL to make the
                    connection to the RabbitMQ server
        :param dict translations: (optional) A dictionary of dictionaries with
                    format translations[SITE][TARGET] = (sensor_1,sensor_2)
        :param str timezone: (optional) A dictionary of sites as keys, with
                   the value as the timezone string for that site.  Assumes
                   all sites are 'America/Los_Angeles' unless given in dict()
        :param int log_level: (optional) The level of logging, LOG_DEBUG=5,
                   LOG_INFO=4, LOG_WARNING=3, LOG_ERROR=2, LOG_CRITICAL=1
        :param str log_file: (optional) The filename for writing the log
        :param bool print_log: (optional) Defines if the logger is printing
                    the output to stdout
        """
        self.name = re.sub('\s', '', str(agent_name))
        self.l = logging.getLogger(__name__).getChild('Connection')

        self._connection = None
        self._channel = None
        self._closing = False
        self._has_on_cancel_callback = False
        
        # To subscribe to an exchange we need:
        #     - an exclusive queue to bind to the exchange
        #       val = "{}.{}".format(self.name, str(uuid.uuid4().hex))
        #     - the exchange type ('topic')
        #     - the exchange durable type (True/False)
        #     - a routing key for the exchange
        #     - a callback function to call when we get a message
        self._exchanges_subscribe = list()
        self._exchanges_publish = list()
        self._queues_subscribe = list()
        self._queues_publish = list()
        
        self._on_connect_callback = None
        self._on_disconnect_callback = None
        
        self.amqp_user = amqp_user
        self.amqp_pass = amqp_pass
        self.amqp_host = amqp_host
        self.amqp_port = amqp_port
        self.amqp_vhost = amqp_vhost
        self.amqp_ssl = amqp_ssl
        self.amqp_url_start = "amqp://"
        if self.amqp_ssl:
            self.amqp_url_start = "amqps://"
        self.translations = copy.deepcopy(translations)
        self.timezone = dict()
        self.timezone['default'] = pytz.timezone('America/Los_Angeles')
        if type(timezone) is dict:
            for key in timezone.keys():
                self.timezone[str(key)] = pytz.timezone(str(timezone[key]))
        
        self._url = "{}{}:{}@{}:{}{}".format(str(self.amqp_url_start),
                                             str(self.amqp_user),
                                             str(self.amqp_pass),
                                             str(self.amqp_host),
                                             str(self.amqp_port),
                                             str(self.amqp_vhost))
        return
    
    def set_on_connect_callback(self, callback):
        self._on_connect_callback = callback
        return
    
    def clear_on_connect_callback(self):
        self._on_connect_callback = None
        return
    
    def set_on_disconnect_callback(self, callback):
        self._on_disconnect_callback = callback
        return
    
    def clear_on_disconnect_callback(self):
        self._on_disconnect_callback = None
        return
    
    def setup_subscribe_to_exchange(self, exchange_name, exchange_type='topic',
                              routing_key='#', exchange_durable=True,
                              casas_events=True, callback_function=None,
                              queue_name=None, queue_durable=False,
                              queue_exclusive=True, queue_auto_delete=True,
                              no_ack=False):
        """
        This function sets up a subscription to events from an exchange.
        
        :param str exchange_name: Name of the RabbitMQ exchange
        :param str exchange_type: The type of the exchnage, usually 'topic'
        :param str routing_key: The routing key to use when binding to the
                   exchange, can be used for filtering by sensor_type
        :param bool exchange_durable: Boolean defining exchange durability
        :param bool casas_events: Boolean defining if this exchange will be
                    sending casas events
        :param function callback_function: The callback function to call when
                        events from this queue binding arrive
        :param str queue_name: Name of the RabbitMQ queue
        :param bool queue_durable: Boolean defining queue durability
        :param bool queue_exclusive: Boolean defining queue exclusivity
        :param bool queue_auto_delete: Boolean defining if the queue should be
                    automatically deleted on consumer disconnection
        """
        self.l.info("setup_subscribe_to_exchange({}, {}, {}, {}, {}, {})".format(str(exchange_name),
                                                 str(exchange_type),
                                                 str(routing_key),
                                                 str(exchange_durable),
                                                 str(casas_events),
                                                 str(callback_function)))
        if callback_function == None:
            self.l.error("casas.rabbitmq.Connection.setup_subscribe_to_exchange(): Error! callback_function needs to be provided for handling events from the exchange.")
            return
        new_sub = dict()
        new_sub['exchange_name'] = exchange_name
        new_sub['exchange_type'] = exchange_type
        new_sub['routing_key'] = routing_key
        new_sub['exchange_durable'] = exchange_durable
        new_sub['casas_events'] = casas_events
        new_sub['callback_function'] = callback_function
        if queue_name == None:
            new_sub['queue_name'] = "{}.{}.{}".format(self.name,
                                                      exchange_name,
                                                      str(uuid.uuid4().hex))
        else:
            new_sub['queue_name'] = queue_name
        new_sub['queue_durable'] = queue_durable
        new_sub['queue_exclusive'] = queue_exclusive
        new_sub['queue_auto_delete'] = queue_auto_delete
        new_sub['no_ack'] = no_ack
        new_sub['consume'] = ConsumeCallback(callback_function=callback_function,
                                             is_exchange=True,
                                             exchange_name=exchange_name,
                                             no_ack=no_ack,
                                             translations=self.translations,
                                             timezone=self.timezone)
        new_sub['consumer_tag'] = ""
        new_sub['setup_exchange'] = False
        new_sub['setup_queue'] = False
        self._exchanges_subscribe.append(new_sub)
        self._setup_all_exchanges()
        return
    
    def remove_subscribe_to_exchange(self, exchange_name, routing_key):
        self.l.info('remove_subscribe_to_exchange({}, {})'.format(str(exchange_name),
                                                                  str(routing_key)))
        if self._channel:
            for ex in self._exchanges_subscribe:
                if ex['exchange_name'] == exchange_name and ex['routing_key'] == routing_key:
                    if ex['setup_exchange'] and ex['setup_queue']:
                        self._channel.queue_unbind(callback=None,
                                                   queue=ex['queue_name'],
                                                   exchange=ex['exchange_name'],
                                                   routing_key=ex['routing_key'])
                        self._channel.basic_cancel(callback=None,
                                                   consumer_tag=ex['consumer_tag'],
                                                   nowait=True)
                        del ex
        return
    
    def setup_publish_to_exchange(self, exchange_name, exchange_type='topic',
                                  exchange_durable=True, routing_key=None):
        new_pub = dict()
        new_pub['exchange_name'] = exchange_name
        new_pub['exchange_type'] = exchange_type
        new_pub['exchange_durable'] = exchange_durable
        new_pub['routing_key'] = routing_key
        new_pub['exchange_setup'] = False
        self._exchanges_publish.append(new_pub)
        self._setup_all_exchanges()
        return
    
    def remove_publish_to_exchange(self, exchange_name):
        for ex in self._exchanges_publish:
            if ex['exchange_name'] == exchange_name:
                del ex
        return
    
    def setup_subscribe_to_queue(self, queue_name, queue_durable=False,
                                 queue_exclusive=False, queue_auto_delete=False,
                                 casas_events=True, callback_function=None,
                                 limit_to_sensor_types=list(), no_ack=False):
        """
        This function sets up a subscription to events from a queue.
        
        :param str queue_name: Name of the RabbitMQ queue
        :param bool queue_durable: Boolean defining queue durability
        :param bool queue_exclusive: Boolean defining queue exclusivity
        :param bool queue_auto_delete: Boolean defining if the queue should be
                    automatically deleted on consumer disconnection
        :param bool casas_events: Boolean defining if this queue will be
                    sending casas events
        :param function callback_function: The callback function to call when
                        events from this queue arrive
        :param list limit_to_sensor_types: (optional) A list of sensor types
                    to limit sending to the callback function, an empty list
                    results in no filtering of events
        """
        if callback_function == None:
            self.l.error("casas.rabbitmq.Connection.setup_subscribe_to_queue(): Error! callback_function needs to be provided for handling events from the queue.")
            return
        new_sub = dict()
        new_sub['queue_name'] = queue_name
        new_sub['queue_durable'] = queue_durable
        new_sub['queue_exclusive'] = queue_exclusive
        new_sub['queue_auto_delete'] = queue_auto_delete
        new_sub['casas_events'] = casas_events
        new_sub['callback_function'] = callback_function
        new_sub['limit_to_sensor_types'] = copy.deepcopy(limit_to_sensor_types)
        new_sub['no_ack'] = no_ack
        new_sub['consume'] = ConsumeCallback(callback_function=callback_function,
                                             is_queue=True,
                                             queue_name=queue_name,
                                             limit_to_sensor_types=limit_to_sensor_types,
                                             no_ack=no_ack,
                                             translations=self.translations,
                                             timezone=self.timezone)
        new_sub['consumer_tag'] = ""
        new_sub['setup_queue'] = False
        self._queues_subscribe.append(new_sub)
        self._setup_all_queues()
        return
    
    def remove_subscribe_to_queue(self, queue_name, limit_to_sensor_types=list()):
        self.l.info('remove_subscribe_to_queue({}, {})'.format(str(queue_name),
                                                               str(limit_to_sensor_types)))
        if self._channel:
            for qu in self._queues_subscribe:
                if qu['queue_name'] == queue_name and sorted(qu['limit_to_sensor_types']) == sorted(limit_to_sensor_types):
                    if qu['setup_queue']:
                        self._channel.basic_cancel(callback=None,
                                                   consumer_tag=qu['consumer_tag'],
                                                   nowait=True)
                        del qu
        return
    
    def setup_publish_to_queue(self, queue_name, queue_durable=False,
                               queue_exclusive=False, queue_auto_delete=False,
                               delivery_mode=2):
        new_pub = dict()
        new_pub['queue_name'] = queue_name
        new_pub['queue_durable'] = queue_durable
        new_pub['queue_exclusive'] = queue_exclusive
        new_pub['queue_auto_delete'] = queue_auto_delete
        new_pub['delivery_mode'] = 2
        new_pub['setup_queue'] = False
        self._queues_publish.append(new_pub)
        self._setup_all_queues()
        return
    
    def remove_publish_to_queue(self, queue_name):
        for qu in self._queues_publish:
            if qu['queue_name'] == queue_name:
                del qu
        return

    def _connect(self):
        """
        This method connects to RabbitMQ, returning the connection handle.
        When the connection is established, the _on_connection_open method
        will be invoked by pika.

        :rtype: pika.SelectConnection

        """
        self.l.info('Connecting to %s', self._url)
        return pika.SelectConnection(pika.URLParameters(self._url),
                                     self._on_connection_open,
                                     stop_ioloop_on_close=False)

    def _on_connection_open(self, unused_connection):
        """
        This method is called by pika once the connection to RabbitMQ has
        been established. It passes the handle to the connection object in
        case we need it, but in this case, we'll just mark it unused.

        :type unused_connection: pika.SelectConnection

        """
        self.l.info('Connection opened')
        self._add_on_connection_close_callback()
        self._open_channel()
        if self._on_connect_callback != None:
            self.call_later(1, self._on_connect_callback)
        return

    def _add_on_connection_close_callback(self):
        """
        This method adds an on close callback that will be invoked by pika
        when RabbitMQ closes the connection to the publisher unexpectedly.

        """
        self.l.info('Adding connection close callback')
        self._connection.add_on_close_callback(self._on_connection_closed)
        return

    def _on_connection_closed(self, connection, reply_code, reply_text):
        """
        This method is invoked by pika when the connection to RabbitMQ is
        closed unexpectedly. Since it is unexpected, we will reconnect to
        RabbitMQ if it disconnects.

        :param pika.connection.Connection connection: The closed connection obj
        :param int reply_code: The server provided reply_code if given
        :param str reply_text: The server provided reply_text if given

        """
        self._channel = None
        self._has_on_cancel_callback = False
        for ex in self._exchanges_subscribe:
            ex['setup_exchange'] = False
            ex['setup_queue'] = False
        for ex in self._exchanges_publish:
            ex['setup_exchange'] = False
        for qu in self._queues_subscribe:
            qu['setup_queue'] = False
        for qu in self._queues_publish:
            qu['setup_queue'] = False
        
        if self._closing:
            self._connection.ioloop.stop()
        else:
            self.l.warning('Connection closed, reopening in 5 seconds: (%s) %s',
                           reply_code, reply_text)
            self._connection.add_timeout(5, self._reconnect)
            if self._on_disconnect_callback != None:
                self.call_later(1, self._on_disconnect_callback)
        return

    def _reconnect(self):
        """
        Will be invoked by the IOLoop timer if the connection is
        closed. See the on_connection_closed method.

        """
        # This is the old connection IOLoop instance, stop its ioloop
        self._connection.ioloop.stop()

        if not self._closing:

            # Create a new connection
            self._connection = self._connect()

            # There is now a new connection, needs a new ioloop to run
            self._connection.ioloop.start()
        return

    def _open_channel(self):
        """
        Open a new channel with RabbitMQ by issuing the Channel.Open RPC
        command. When RabbitMQ responds that the channel is open, the
        on_channel_open callback will be invoked by pika.

        """
        self.l.info('Creating a new channel')
        self._connection.channel(on_open_callback=self._on_channel_open)
        return

    def _on_channel_open(self, channel):
        """
        This method is invoked by pika when the channel has been opened.
        The channel object is passed in so we can make use of it.

        Since the channel is now open, we'll declare the exchange to use.

        :param pika.channel.Channel channel: The channel object

        """
        self.l.info('Channel opened')
        self._channel = channel
        self._add_on_channel_close_callback()
        self._setup_all_exchanges()
        self._setup_all_queues()
        return

    def _add_on_channel_close_callback(self):
        """
        This method tells pika to call the on_channel_closed method if
        RabbitMQ unexpectedly closes the channel.

        """
        self.l.info('Adding channel close callback')
        self._channel.add_on_close_callback(self._on_channel_closed)
        return

    def _on_channel_closed(self, channel, reply_code, reply_text):
        """
        Invoked by pika when RabbitMQ unexpectedly closes the channel.
        Channels are usually closed if you attempt to do something that
        violates the protocol, such as re-declare an exchange or queue with
        different parameters. In this case, we'll close the connection
        to shutdown the object.

        :param pika.channel.Channel: The closed channel
        :param int reply_code: The numeric reason the channel was closed
        :param str reply_text: The text reason the channel was closed

        """
        self.l.warning('Channel %i was closed: (%s) %s',
                       channel, reply_code, reply_text)
        self._connection.close()
        return
    
    def _setup_all_exchanges(self):
        """
        Setup the exchanges on RabbitMQ by invoking self.setup_exchange(...)
        for each exchange we have defined.
        """
        if self._channel:
            self.l.info('Setting up defined exchanges')
            for ex in self._exchanges_subscribe:
                self.l.info("setting up: {}".format(str(ex['exchange_name'])))
                self._setup_exchange(ex)
            for ex in self._exchanges_publish:
                self.l.info("setting up: {}".format(str(ex['exchange_name'])))
                self._setup_exchange(ex)
            self._add_on_cancel_callback()
        return

    def _setup_exchange(self, ex):
        """
        Setup the exchange on RabbitMQ by invoking the Exchange.Declare RPC
        command. When it is complete, the on_exchange_declareok method will
        be invoked by pika.

        :param dict ex: The dict of the exchange to start setting up

        """
        try:
            if not ex['setup_exchange']:
                self.l.info('Declaring exchange %s', ex['exchange_name'])
                self._channel.exchange_declare(callback=None,
                                               exchange=ex['exchange_name'],
                                               exchange_type=ex['exchange_type'],
                                               durable=ex['exchange_durable'])
                ex['setup_exchange'] = True
                if 'consume' in ex:
                    self.l.info('Declaring queue %s', ex['queue_name'])
                    self._channel.queue_declare(callback=None,
                                                queue=ex['queue_name'],
                                                durable=ex['queue_durable'],
                                                exclusive=ex['queue_exclusive'],
                                                auto_delete=ex['queue_auto_delete'])
                    self._channel.queue_bind(callback=None,
                                             exchange=ex['exchange_name'],
                                             queue=ex['queue_name'],
                                             routing_key=ex['routing_key'])
                    ex['setup_queue'] = True
                    ex['consumer_tag'] = self._channel.basic_consume(consumer_callback=ex['consume'].on_message,
                                                                     queue=ex['queue_name'],
                                                                     no_ack=ex['no_ack'])
        except pika.exceptions.AMQPChannelError:
            self.l.error("casas.rabbitmq.Connection.setup_exchange(): pika.exceptions.AMQPChannelError, {}".format(ex['exchange_name']))
            self.l.error("  name={} type={} durable={}".format(str(ex['exchange_name']),
                                                               str(ex['exchange_type']),
                                                               str(ex['exchange_durable'])))
        return

    def _setup_all_queues(self):
        """
        Setup the queues on RabbitMQ by invoking self.setup_queue(...) for
        each queue we have defined.
        """
        if self._channel:
            self.l.info('Setting up defined queues')
            for qu in self._queues_subscribe:
                self.l.info("setting up: {}".format(str(qu['queue_name'])))
                self._setup_queue(qu)
            for qu in self._queues_publish:
                self.l.info("setting up: {}".format(str(qu['queue_name'])))
                self._setup_queue(qu)
            self._add_on_cancel_callback()
        return

    def _setup_queue(self, qu):
        """
        Setup the queue on RabbitMQ by invoking the Queue.Declare RPC
        command.

        :param dict qu: The dict of the queue to start setting up

        """
        try:
            if not qu['setup_queue']:
                self.l.info('Declaring queue %s', qu['queue_name'])
                self._channel.queue_declare(callback=None,
                                            queue=qu['queue_name'],
                                            durable=qu['queue_durable'],
                                            exclusive=qu['queue_exclusive'],
                                            auto_delete=qu['queue_auto_delete'])
                qu['setup_queue'] = True
                if 'consume' in qu:
                    qu['consumer_tag'] = self._channel.basic_consume(consumer_callback=qu['consume'].on_message,
                                                                     queue=qu['queue_name'],
                                                                     no_ack=qu['no_ack'])
        except pika.exceptions.AMQPChannelError:
            self.l.error("setup_queue(): pika.exceptions.AMQPChannelError, {}".format(qu['queue_name']))
            self.l.error("  name={} exclusive={} durable={}".format(str(qu['queue_name']),
                                                                    str(qu['queue_exclusive']),
                                                                    str(qu['queue_durable'])))
        return
    
    def publish_to_exchange(self, exchange_name, casas_object=None,
                            body_str=None, routing_key=None,
                            correlation_id=None, key=None, secret=None):
        if routing_key == None:
            if casas_object == None:
                routing_key = "unknown.unknown.casas"
            else:
                routing_key = "{}.{}.casas".format(str(casas_object.sensor_type),
                                                   str(casas_object.site))
        if correlation_id == None:
            correlation_id = str(uuid.uuid4())
        
        if self._channel:
            if casas_object != None:
                body_str = casas_object.get_json(secret=secret, key=key)
            self.l.debug("publish_to_exchange(exchange=%s, casas_obj=%s, body=%s, routing_key=%s, corr_id=%s, key=%s, secret=%s)",
                         str(exchange_name), str(casas_object), str(body_str),
                         str(routing_key), str(correlation_id), str(key),
                         str(secret))
            self._channel.basic_publish(exchange=exchange_name,
                                        routing_key=routing_key,
                                        properties=pika.BasicProperties(correlation_id=correlation_id),
                                        body=str(body_str))
        return
    
    def publish_to_queue(self, queue_name, casas_object=None, body_str=None,
                         correlation_id=None, delivery_mode=2, key=None,
                         secret=None):
        if correlation_id == None:
            correlation_id = str(uuid.uuid4())
        
        if self._channel:
            if casas_object != None:
                body_str = casas_object.get_json(secret=secret, key=key)
                body_str = "[{}]".format(body_str)
            #self.l.info("publish_to_queue({}, {})".format(str(queue_name),
            #                                              str(body_str)))
            self.l.debug("publish_to_queue(queue=%s, casas_obj=%s, body=%s, corr_id=%s, del_mode=%s, key=%s, secret=%s)",
                         str(queue_name), str(casas_object), str(body_str),
                         str(correlation_id), str(delivery_mode), str(key),
                         str(secret))
            self._channel.basic_publish(exchange='',
                                        routing_key=queue_name,
                                        properties=pika.BasicProperties(
                                                correlation_id=correlation_id,
                                                delivery_mode=delivery_mode),
                                        body=str(body_str))
        return

    def _add_on_cancel_callback(self):
        """
        Add a callback that will be invoked if RabbitMQ cancels the consumer
        for some reason. If RabbitMQ does cancel the consumer,
        on_consumer_cancelled will be invoked by pika.
        """
        if not self._has_on_cancel_callback:
            self.l.info('Adding consumer cancellation callback')
            self._channel.add_on_cancel_callback(self._on_consumer_cancelled)
            self._has_on_cancel_callback = True
        return

    def _on_consumer_cancelled(self, method_frame=None):
        """
        Invoked by pika when RabbitMQ sends a Basic.Cancel for a consumer
        receiving messages.

        :param pika.frame.Method method_frame: The Basic.Cancel frame

        """
        self.l.info('Consumer was cancelled remotely, shutting down: %s',
                    str(method_frame))
        if self._channel:
            self._channel.close()
            self._connection.close()
        return

    def _stop_consuming(self):
        """
        Tell RabbitMQ that you would like to stop consuming by sending the
        Basic.Cancel RPC command.

        """
        if self._channel:
            self.l.info('Sending a Basic.Cancel RPC commands to RabbitMQ')
            for ex in self._exchanges_subscribe:
                if not ex['setup_exchange'] and not ex['setup_queue']:
                    continue
                
                self._channel.basic_cancel(callback=None,
                                           consumer_tag=ex['consumer_tag'],
                                           nowait=True)
            
            for qu in self._queues_subscribe:
                if not qu['setup_queue']:
                    continue
                
                self._channel.basic_cancel(callback=None,
                                           consumer_tag=qu['consumer_tag'],
                                           nowait=True)
            self.call_later(1, self._on_consumer_cancelled)
        return

    def run(self):
        """
        Run the example consumer by connecting to RabbitMQ and then
        starting the IOLoop to block and allow the SelectConnection to operate.

        """
        self._connection = self._connect()
        self._connection.ioloop.start()
        return

    def stop(self):
        """
        Cleanly shutdown the connection to RabbitMQ by stopping the consumer
        with RabbitMQ. When RabbitMQ confirms the cancellation, on_cancelok
        will be invoked by pika, which will then closing the channel and
        connection. The IOLoop is started again because this method is invoked
        when CTRL-C is pressed raising a KeyboardInterrupt exception. This
        exception stops the IOLoop which needs to be running for pika to
        communicate with RabbitMQ. All of the commands issued prior to starting
        the IOLoop will be buffered but not processed.

        """
        if not self._closing:
            self.l.info('Stopping')
            self._closing = True
            self._stop_consuming()
            self._connection.ioloop.start()
            self.l.info('Stopped')
        return
    
    def call_later(self, seconds, function):
        """
        Adds a callback to the IO loop.
        Returns the timeout ID string.
        """
        return self._connection.add_timeout(seconds, function)
    
    def cancel_call_later(self, timeout_id):
        self._connection.remove_timeout(timeout_id)
        return



