#*****************************************************************************#
#**
#**  CASAS XMPP Python Library
#**
#**    Brian L Thomas, 2014
#**
#** Tools by the Center for Advanced Studies in Adaptive Systems at
#**  the School of Electrical Engineering and Computer Science at
#**  Washington State University
#** 
#** Copyright Washington State University, 2014
#** Copyright Brian L. Thomas, 2014
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
import sys

if "pygtk" in sys.modules:
    from twisted.internet import gtk2reactor
    gtk2reactor.install()

from twisted.words.protocols.jabber import client, jid, xmlstream
from twisted.words.xish import domish
from twisted.internet import reactor

import copy
import datetime
import json
import re
import signal
import time
import uuid
import xml.dom.minidom
import xml.etree.ElementTree as ETree
import xml.parsers.expat

def getText(nodelist):
    rc = ""
    for node in nodelist:
        if node.nodeType == node.TEXT_NODE:
            rc = rc + node.data
    return str(rc)

def getData(node, message):
    result = ""
    try:
        dom = xml.dom.minidom.parseString(message)
        xmsg = dom.getElementsByTagName(node)
        if xmsg.length > 0:
            result = getText(xmsg[0].childNodes)
    except xml.parsers.expat.ExpatError:
        result = ""
    return result

def dict_to_xml(oDict):
    if len(oDict) != 1:
        print "ERROR: dict_to_xml() needs dict() of len=1"
        return None
    key = oDict.keys()[0]
    root = ETree.Element(key)
    dict_to_xml_children(oDict[key], key, root)
    return root

def dict_to_xml_children(obj, name, xElem):
    if type(obj) == type(list()):
        for dup in obj:
            if type(dup) == type(dict()):
                elem = ETree.SubElement(xElem, name)
                dict_to_xml_children(dup, name, elem)
            else:
                dict_to_xml_children(dup, name, xElem)
    elif type(obj) == type(dict()):
        for key in obj.keys():
            if type(obj[key]) != type(list()):
                elem = ETree.SubElement(xElem, key)
                dict_to_xml_children(obj[key], key, elem)
            else:
                for dup in obj[key]:
                    elem = ETree.SubElement(xElem, key)
                    dict_to_xml_children(dup, key, elem)
    elif type(obj) == type(str()):
        xElem.text = obj
    return

def xmlstr_to_dict(message):
    try:
        xElem = ETree.fromstring(message)
        newDict = xml_to_dict(xElem, dict())
    except xml.parsers.expat.ExpatError:
        newDict = dict()
    except xml.etree.ElementTree.ParseError:
        newDict = dict()
    return newDict

def xml_to_dict(xElem, newDict=dict()):
    position = None
    if xElem.tag in newDict:
        if type(newDict[xElem.tag]) != type(list()):
            newDict[xElem.tag] = [newDict[xElem.tag]]
        position = len(newDict[xElem.tag])
        if xElem.text:
            newDict[xElem.tag].append(xElem.text)
        else:
            newDict[xElem.tag].append("")
    else:
        if xElem.text:
            newDict[xElem.tag] = xElem.text
        else:
            newDict[xElem.tag] = dict()
    children = xElem.getchildren()
    if children:
        if position == None:
            newDict[xElem.tag] = dict()
        else:
            newDict[xElem.tag][position] = dict()
        for child in children:
            if position == None:
                newDict[xElem.tag] = xml_to_dict(child, newDict[xElem.tag])
            else:
                newDict[xElem.tag][position] = xml_to_dict(child, newDict[xElem.tag][position])
    if newDict[xElem.tag] == dict():
        newDict[xElem.tag] = ""
    return newDict

def make_epoch(stamp):
    epoch = time.mktime(stamp.timetuple())
    #epoch = float(epoch) + (float(datetime_obj.microsecond)/1000000.0)
    epoch = float(epoch) + float("0." + str(stamp).split('.')[1])
    return epoch

def getJustText(node):
    result = ""
    if node.length > 0:
        result = getText(node[0].childNodes)
    return result

def build_events_from_statesummary(data):
    events = list()
    dom = xml.dom.minidom.parseString(data)
    xdevice = dom.getElementsByTagName("device")
    for device in xdevice:
        e = Event("")
        e.by = getJustText(device.getElementsByTagName("by"))
        e.category = getJustText(device.getElementsByTagName("category"))
        e.epoch = getJustText(device.getElementsByTagName("epoch"))
        e.target = getJustText(device.getElementsByTagName("target"))
        e.message = getJustText(device.getElementsByTagName("message"))
        e.serial = getJustText(device.getElementsByTagName("serial"))
        e.sensor_type = getJustText(device.getElementsByTagName("sensortype"))
        e.package_type = getJustText(device.getElementsByTagName("packagetype"))
        e.uuid = getJustText(device.getElementsByTagName("uuid"))
        if e.epoch == "":
            e.stamp = datetime.datetime.now()
            e.epoch = str(time.time())
        else:
            e.stamp = datetime.datetime.fromtimestamp(float(e.epoch))
        events.append(e)
    return events


class Data:
    def clean(self, msg):
        msg = re.sub('_#!#_', '', msg)
        return msg
    
    def get(self, obj, keys):
        val = ""
        if len(keys) > 1:
            if keys[0] in obj:
                val = self.get(obj[keys[0]], keys[1:])
        elif len(keys) > 0:
            if keys[0] in obj:
                val = obj[keys[0]]
        return val


class Event(Data):
    by = ""
    category = ""
    stamp = ""
    epoch = ""
    target = ""
    message = ""
    serial = ""
    sensor_type = ""
    package_type = ""
    uuid = ""
    channel = ""
    def __init__(self, msg):
        data = xmlstr_to_dict(msg)
        if data != dict():
            self.build_event(data)
        return
    
    def build_event(self, data):
        self.by = self.clean(self.get(data, ['publish','data','event','by']))
        self.category = self.clean(self.get(data, ['publish','data','event','category']))
        self.epoch = self.clean(self.get(data, ['publish','data','event','epoch']))
        self.target = self.clean(self.get(data, ['publish','data','event','target']))
        self.message = self.clean(self.get(data, ['publish','data','event','message']))
        self.serial = self.clean(self.get(data, ['publish','data','event','serial']))
        self.sensor_type = self.clean(self.get(data, ['publish','data','event','sensortype']))
        self.package_type = self.clean(self.get(data, ['publish','data','event','packagetype']))
        self.uuid = self.clean(self.get(data, ['publish','data','event','uuid']))
        self.channel = self.clean(self.get(data, ['publish','channel']))
        if self.by == "":
            self.by = "unknown"
        if self.category == "":
            self.category = "unknown"
        if self.epoch == "":
            self.stamp = datetime.datetime.now()
            self.epoch = str(time.time())
        else:
            self.stamp = datetime.datetime.fromtimestamp(float(self.epoch))
        if self.target == "":
            self.target = "unknown"
        if self.message == "":
            self.message = "unknown"
        if self.serial == "":
            self.serial = "unknown"
        if self.sensor_type == "":
            self.sensor_type = "unknown"
        if self.package_type == "":
            self.package_type = "unknown"
        if self.uuid == "":
            self.uuid = str(uuid.uuid4().hex)
        if self.channel == "":
            self.channel = "unknown"
        return
    
    def get_scribe_event(self):
        msg = "EVENT"
        msg += "_#!#_%s" % self.uuid
        msg += "_#!#_%s" % str(self.stamp)
        msg += "_#!#_%s" % self.serial
        msg += "_#!#_%s" % self.target
        msg += "_#!#_%s" % self.message
        msg += "_#!#_%s" % self.by
        msg += "_#!#_%s" % self.category
        msg += "_#!#_%s" % self.sensor_type
        msg += "_#!#_%s" % self.package_type
        return msg
    
    def get_json(self, site="casas", secret="secret", key="key"):
        obj = {"channel":self.channel,
               "site":site,
               "secret":secret,
               "key":key,
               "action":"event",
               "data":{"uuid":self.uuid,
                       "epoch":self.epoch,
                       "serial":self.serial,
                       "target":self.target,
                       "message":self.message,
                       "by":self.by,
                       "category":self.category,
                       "sensor_type":self.sensor_type,
                       "package_type":self.package_type}}
        return json.dumps(obj)


class Tag(Event):
    createdBy = ""
    labelName = ""
    dsetName = ""
    expirName = ""
    def __init__(self, msg):
        data = xmlstr_to_dict(msg)
        if data != dict():
            self.build_event(data)
            self.build_tag(data)
        return
    
    def build_tag(self, data):
        self.createdBy = self.clean(self.get(data, ['publish','data','tag','created_by']))
        self.labelName = self.clean(self.get(data, ['publish','data','tag','label','name']))
        self.labelValue = self.clean(self.get(data, ['publish','data','tag','label','value']))
        self.dsetName = self.clean(self.get(data, ['publish','data','tag','dataset','name']))
        self.expirName = self.clean(self.get(data, ['publish','data','tag','experiment','name']))
        if self.createdBy == "":
            self.createdBy = "unknown"
        if self.dsetName == "":
            self.dsetName = "unknown"
        if self.expirName == "":
            self.expirName = "unknown"
        return
    
    def get_scribe_tag(self):
        msg = "TAG"
        msg += "_#!#_%s" % self.createdBy
        msg += "_#!#_%s" % self.labelName
        msg += "_#!#_%s" % self.labelValue
        msg += "_#!#_%s" % self.dsetName
        msg += "_#!#_%s" % self.expirName
        msg += "_#!#_%s" % self.uuid
        msg += "_#!#_%s" % str(self.stamp)
        msg += "_#!#_%s" % self.serial
        msg += "_#!#_%s" % self.target
        msg += "_#!#_%s" % self.message
        msg += "_#!#_%s" % self.by
        msg += "_#!#_%s" % self.category
        msg += "_#!#_%s" % self.sensor_type
        msg += "_#!#_%s" % self.package_type
        return msg
    
    def get_json(self, site="casas", secret="secret", key="key"):
        obj = {"channel":self.channel,
               "site":site,
               "secret":secret,
               "key":key,
               "action":"tag",
               "data":{"tag":{"created_by":self.createdBy,
                              "label":{"name":self.labelName,
                                       "value":self.labelValue},
                              "dataset":self.dsetName,
                              "experiment":self.expirName},
                       "uuid":self.uuid,
                       "epoch":self.epoch,
                       "serial":self.serial,
                       "target":self.target,
                       "message":self.message,
                       "by":self.by,
                       "category":self.category,
                       "sensor_type":self.sensor_type,
                       "package_type":self.package_type}}
        return json.dumps(obj)


class Command(Data):
    by = ""
    command = ""
    value = ""
    serial = ""
    target = ""
    replyto = ""
    response = ""
    cid = ""
    response = ""
    stamp = ""
    epoch = ""
    uuid = ""
    channel = ""
    def __init__(self, msg):
        data = xmlstr_to_dict(msg)
        if data != dict():
            self.build_command(data)
        return
    
    def build_command(self, data):
        self.by = self.clean(self.get(data, ['publish','data','control','by']))
        self.command = self.clean(self.get(data, ['publish','data','control','command']))
        self.value = self.clean(self.get(data, ['publish','data','control','value']))
        self.serial = self.clean(self.get(data, ['publish','data','control','serial']))
        self.target = self.clean(self.get(data, ['publish','data','control','target']))
        self.replyto = self.clean(self.get(data, ['publish','data','control','replyto']))
        self.response = self.clean(self.get(data, ['publish','data','control','response']))
        self.cid = self.clean(self.get(data, ['publish','data','control','cid']))
        self.response = self.clean(self.get(data, ['publish','data','control','response']))
        self.epoch = self.clean(self.get(data, ['publish','data','control','epoch']))
        self.uuid = self.clean(self.get(data, ['publish','data','control','uuid']))
        self.channel = self.clean(self.get(data, ['publish','channel']))
        if self.by == "":
            self.by = "unknown"
        if self.epoch == "":
            self.stamp = datetime.datetime.now()
            self.epoch = str(time.time())
        else:
            self.stamp = datetime.datetime.fromtimestamp(float(self.epoch))
        if self.target == "":
            self.target = "unknown"
        if self.serial == "":
            self.serial = "unknown"
        if self.channel == "":
            self.channel = "unknown"
        return
    
    def get_as_event_obj(self):
        e = Event("")
        e.by = str(self.by)
        e.category = "control"
        e.epoch = str(self.epoch)
        e.stamp = copy.deepcopy(self.stamp)
        e.target = str(self.target)
        e.serial = str(self.serial)
        e.sensor_type = "control"
        e.package_type = "control"
        e.uuid = str(self.uuid)
        e.channel = str(self.channel)
        e.message = dict({'command':self.command,
                          'value':self.value,
                          'cid':self.cid,
                          'replyto':self.replyto})
        e.message = str(json.dumps(e.message, sort_keys=True))
        return e
    
    def get_scribe_event(self):
        msg = "EVENT"
        msg += "_#!#_%s" % self.uuid
        msg += "_#!#_%s" % str(self.stamp)
        msg += "_#!#_%s" % self.serial
        msg += "_#!#_%s" % self.target
        msg += "_#!#_%s|%s|%s|%s" % (self.command, self.value, self.replyto, self.cid)
        msg += "_#!#_%s" % self.by
        msg += "_#!#_control"
        msg += "_#!#_control"
        return msg
    
    def get_json(self, site="casas", secret="secret", key="key"):
        obj = {"channel":self.channel,
               "site":site,
               "secret":secret,
               "key":key,
               "action":"control",
               "data":{"uuid":self.uuid,
                       "epoch":self.epoch,
                       "serial":self.serial,
                       "target":self.target,
                       "command":self.command,
                       "value":self.value,
                       "replyto":self.replyto,
                       "cid":self.cid,
                       "by":self.by,
                       "category":"control"}}
        return json.dumps(obj)


class Connection:
    def __init__(self, name):
        self.name = name
        self.isAuthenticated = False
        self.manager = None
        self.jid = None
        self.server = None
        self.chans_subscribe = []
        self.chans_publish = []
        self.authd_callback = None
        self.buddy_quit_callback = None
        self.channel_callback = dict()
        self.direct_msg_callback = None
        self.error_callback = None
        self.finish_callback = None
        self.statesummary_callback = None
        self.reactor = reactor
        self.debugging = False
        self.waiting_roster = False
        self.roster = None
        return
    
    def connect(self, jidVal=None, password=None):
        myJid = jid.JID(jidVal)
        if self.manager == None:
            self.manager = "manager@%s" % myJid.host
        self.jid = myJid.userhost()
        self.server = str(myJid.host)
        myJid.resource = "onlyone"
        factory = client.basicClientFactory(myJid, password)
        factory.addBootstrap('//event/stream/authd', self.authd)
        self.reactor.connectTCP(myJid.host, 5222, factory)
        if self.finish_callback != None:
            print "XMPP Lib: setting finish_callback"
            self.reactor.addSystemEventTrigger('before', signal.SIGINT, self.finish_callback)
        self.reactor.run()
        return
    
    def disconnect(self):
        if self.isAuthenticated:
            self.unpublish_channel("log")
            self.isAuthenticated = False
            if self.reactor.running:
                self.reactor.stop()
        return
    
    def authd(self, xmlstreamobj):
        print "authenticated"
        self.isAuthenticated = True
        
        xmlstreamobj.addObserver('/presence', self.presence)
        xmlstreamobj.addObserver('/message', self.listen)
        xmlstreamobj.addObserver('/iq', self.handle_iq)
        self.connection = xmlstreamobj
        
        self.set_status("%s is up!" % self.name)
        self.subscribe_buddy(self.manager)
        self.publish_channel("log")
        
        self.waiting_roster = True
        iq = domish.Element(('jabber:client', 'iq'))
        iq['type'] = 'get'
        iq.addElement(('jabber:iq:roster', 'query'))
        self.connection.send(iq)
        
        if self.authd_callback != None:
            self.reactor.callLater(1, self.authd_callback)
        return
    
    def clean_jid(self, dirty):
        clean = jid.JID(str(dirty))
        return clean.userhost()
    
    def callLater(self, seconds, function, *args):
        delayCall = self.reactor.callLater(seconds, function, *args)
        return delayCall
    
    def cancelLater(self, delayCall):
        if delayCall.active():
            delayCall.cancel()
        return
    
    def set_custom_manager(self, manager):
        tmp = str(manager).split('/')
        self.manager = str(tmp[0])
        return
    
    def set_authd_callback(self, cb):
        self.authd_callback = cb
        return
    
    def set_direct_msg_callback(self, cb):
        self.direct_msg_callback = cb
        return
    
    def set_channel_callback(self, channel, cb):
        if channel in self.channel_callback and cb == None:
            del self.channel_callback[channel]
        else:
            self.channel_callback[channel] = cb
        return
    
    def set_error_callback(self, cb):
        self.error_callback = cb
        return
    
    def set_finish_callback(self, cb):
        self.finish_callback = cb
        return
    
    def set_buddy_quit_callback(self, cb):
        self.buddy_quit_callback = cb
        return
    
    def set_statesummary_callback(self, cb):
        self.statesummary_callback = cb
        return
    
    def handle_iq(self, elem):
        if self.waiting_roster and elem.query.uri == 'jabber:iq:roster':
            print "got roster!"
            self.waiting_roster = False
            self.handle_roster(elem)
        print "=" * 60
        print elem.toXml()
        return
    
    def handle_roster(self, elem):
        self.roster = dict()
        items = re.findall('<item .*?</item>|<item .*?/>', elem.toXml())
        for x in items:
            jid = re.findall("jid='(.*?)'", x)
            print jid[0]
            if re.search("ask='", x):
                ask = re.findall("ask='(.*?)'", x)
                print "    ask =",ask[0]
            if re.search("subscription='", x):
                subscr = re.findall("subscription='(.*?)'", x)
                print "    subscription =",subscr[0]
        return
    
    def presence(self, elem):
        if elem.hasAttribute('type'):
            presence = domish.Element(('jabber:client', 'presence'))
            presence['to'] = str(elem['from'])
            presence['from'] = str(self.jid)
            if elem['type'] == "subscribe":
                presence['type'] = "subscribed"
                self.connection.send(presence)
                #p2 = domish.Element(('jabber:client','presence'))
                #p2['to'] = str(elem['from'])
                #p2['from'] = str(self.jid)
                #p2['type'] = "subscribe"
                #self.connection.send(p2)
            elif elem['type'] == "unsubscribe":
                presence['type'] = "unsubscribed"
                self.connection.send(presence)
            elif elem['type'] == "unavailable":
                print "%s went byby..." % str(elem['from'])
                if self.buddy_quit_callback != None:
                    self.buddy_quit_callback(str(elem['from']))
        else:
            if self.clean_jid(elem['from']) == self.manager:
                print "manager (online","#"*40
                self.handle_manager_online()
            else:
                print "presence (online)","="*40
                print elem.toXml()
        return
    
    def handle_manager_online(self):
        for x in self.chans_subscribe:
            self.do_pubsub("register", "subscriber", x)
        for x in self.chans_publish:
            self.do_pubsub("register", "publisher", x)
        return
    
    def listen(self, elem):
        isError = False
        if elem.hasAttribute('type'):
            if elem['type'] == "error":
                isError = True
                if self.error_callback != None:
                    self.error_callback(str(elem.toXml()))
                #if re.search("error code='503'",str(elem.toXml())):
                #    print "ERROR: Couldn't send to %s" % str(elem['from'])
        if not isError:
            if self.debugging:
                print "msg from:",str(elem['from'])
                print str(elem.toXml())
            whofrom = self.clean_jid(elem['from'])
            body = getData("body", str(elem.toXml()))
            if body != "":
                if re.search("<error>.*?</error>", body):
                    if self.error_callback != None:
                        self.error_callback(body)
                    else:
                        print body
                elif whofrom == self.manager:
                    channel = getData("channel", body)
                    if channel in self.channel_callback:
                        self.channel_callback[channel](body)
                    elif re.search("<statesummary>.*?</statesummary>", body):
                        if self.statesummary_callback != None:
                            self.statesummary_callback(body)
                else:
                    if self.direct_msg_callback != None:
                        self.direct_msg_callback(body, self.clean_jid(elem['from']))
        return
    
    def send(self, message, to):
        if self.isAuthenticated:
            msg = domish.Element((None, 'message'))
            msg['to'] = str(to)
            msg['from'] = str(self.jid)
            msg.addElement('body', content=message)
            self.connection.send(msg)
        return
    
    def publish_data(self, by, package_type, sensor_type, serial, target, message, category, channel="rawevents"):
        msg = "<event><by>%s</by>" % str(by)
        msg += "<packagetype>%s</packagetype>" % str(package_type)
        msg += "<sensortype>%s</sensortype>" % str(sensor_type)
        msg += "<serial>%s</serial>" % str(serial)
        msg += "<target>%s</target>" % str(target)
        msg += "<message>%s</message>" % str(message)
        msg += "<category>%s</category>" % str(category)
        msg += "</event>"
        self.publish(msg, channel)
        return
    
    def publish_command(self, by, command, value, serial, target, replyto="", cid="", channel="control"):
        msg = "<control><by>%s</by>" % str(by)
        msg += "<command>%s</command>" % str(command)
        msg += "<value>%s</value>" % str(value)
        msg += "<serial>%s</serial>" % str(serial)
        msg += "<target>%s</target>" % str(target)
        msg += "<replyto>%s</replyto>" % str(replyto)
        msg += "<cid>%s</cid></control>" % str(cid)
        self.publish(msg, channel)
        return
    
    def reply_command(self, cmd, response=""):
        if cmd.replyto == "":
            return
        msg = "<publish><channel>%s</channel><data>" % "control"
        msg += "<control><by>%s</by>" % str(cmd.by)
        msg += "<command>%s</command>" % str(cmd.command)
        msg += "<value>%s</value>" % str(cmd.value)
        msg += "<response>%s</response>" % str(response)
        msg += "<serial>%s</serial>" % str(cmd.serial)
        msg += "<target>%s</target>" % str(cmd.target)
        msg += "<epoch>%s</epoch>" % str(make_epoch(cmd.stamp))
        msg += "<uuid>%s</uuid>" % str(cmd.uuid)
        msg += "<replyto>%s</replyto>" % str(cmd.replyto)
        msg += "<cid>%s</cid></control>" % str(cmd.cid)
        msg += "</data></publish>"
        self.send(msg, cmd.replyto)
        return
    
    def publish(self, message, channel="rawevents"):
        newMsg = "<publish><channel>%s</channel><data>" % str(channel)
        newMsg += message
        newMsg += "</data></publish>"
        self.send(newMsg, self.manager)
        return
    
    def replay_event(self, by, package_type, sensor_type, serial, target, message, category, epoch, uuid="", channel="replay"):
        msg = "<event><by>%s</by>" % str(by)
        msg += "<packagetype>%s</packagetype>" % str(package_type)
        msg += "<sensortype>%s</sensortype>" % str(sensor_type)
        msg += "<serial>%s</serial>" % str(serial)
        msg += "<target>%s</target>" % str(target)
        msg += "<message>%s</message>" % str(message)
        msg += "<category>%s</category>" % str(category)
        msg += "<epoch>%s</epoch>" % str(epoch)
        msg += "<uuid>%s</uuid>" % str(uuid)
        msg += "</event>"
        self.publish(msg, channel)
        return
    
    def tag_event(self, created_by, experiment, dataset, label, value="", by="", package_type="", sensor_type="", serial="", target="", message="", category="", epoch="", uuid="", channel="tag"):
        msg = "<tag>"
        msg += "<created_by>%s</created_by>" % str(created_by)
        msg += "<label><name>%s</name>" % str(label)
        msg += "<value>%s</value></label>" % str(value)
        msg += "<dataset><name>%s</name></dataset>" % str(dataset)
        msg += "<experiment><name>%s</name></experiment>" % str(experiment)
        msg += "</tag>"
        test_event = by + sensor_type + package_type + serial + target + message + category + epoch
        if test_event != "":
            msg += "<event><by>%s</by>" % str(by)
            msg += "<packagetype>%s</packagetype>" % str(package_type)
            msg += "<sensortype>%s</sensortype>" % str(sensor_type)
            msg += "<serial>%s</serial>" % str(serial)
            msg += "<target>%s</target>" % str(target)
            msg += "<message>%s</message>" % str(message)
            msg += "<category>%s</category>" % str(category)
            msg += "<epoch>%s</epoch>" % str(epoch)
            msg += "<uuid>%s</uuid>" % str(uuid)
            msg += "</event>"
        else:
            if channel == "tag":
                if self.error_callback != None:
                    self.error_callback("Publishing to 'tag' requires <event>!")
                else:
                    print "ERROR: Publishing to 'tag' requires <event>!"
                return
        self.publish(msg, channel)
        return
    
    def subscribe_channel(self, channel):
        if channel not in self.chans_subscribe:
            self.chans_subscribe.append(channel)
            self.do_pubsub("register", "subscriber", channel)
        return
    
    def unsubscribe_channel(self, channel):
        if channel in self.chans_subscribe:
            self.chans_subscribe.remove(channel)
            self.do_pubsub("unregister", "subscriber", channel)
        return
    
    def publish_channel(self, channel):
        if channel not in self.chans_publish:
            self.chans_publish.append(channel)
            self.do_pubsub("register", "publisher", channel)
        return
    
    def unpublish_channel(self, channel):
        if channel in self.chans_publish:
            self.chans_publish.remove(channel)
            self.do_pubsub("unregister", "publisher", channel)
        return
    
    def do_pubsub(self, action, who, channel):
        msg = "<system><do>%s</do><as>%s</as>" % (str(action), str(who))
        msg += "<channel>%s</channel></system>" % str(channel)
        self.send(msg, self.manager)
        return
    
    def subscribe_buddy(self, buddy):
        presence = domish.Element(('jabber:client', 'presence'))
        presence['to'] = str(buddy)
        presence['from'] = str(self.jid)
        presence['type'] = "subscribe"
        self.connection.send(presence)
        return
    
    def unsubscribe_buddy(self, buddy):
        presence = domish.Element(('jabber:client', 'presence'))
        presence['to'] = str(buddy)
        presence['from'] = str(self.jid)
        presence['type'] = "unsubscribe"
        self.connection.send(presence)
        return
    
    def request_statesummary(self):
        self.send("<requeststate></requeststate>", self.manager)
        return
    
    def log(self, message):
        self.publish_data(self.name, "system", "system", self.name, "system", 
                          message, "system", "log")
        return
    
    def log_finish(self, message):
        print self.name,"  msg:",message
        return
    
    def debug(self, message):
        self.publish_data(self.name, "system", "system", self.name, "system",
                          message, "system", "debug")
        return
    
    def set_status(self, value):
        status = domish.Element(('jabber:client', 'presence'))
        status.addElement('status', content=value)
        self.connection.send(status)
        return

