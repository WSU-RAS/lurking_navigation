#*****************************************************************************#
#**
#**  CASAS RabbitMQ JSON Object Python Library
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
import re
import time
import uuid


l = logging.getLogger(__name__)



class Event(object):
    action = "event"
    category = ""
    package_type = ""
    sensor_type = ""
    message = ""
    target = ""
    serial = ""
    by = ""
    channel = ""
    site = ""
    stamp = ""
    stamp_local = ""
    epoch = ""
    uuid = ""
    site = ""
    sensor_1 = ""
    sensor_2 = ""
    
    def __init__(self, category, package_type, sensor_type, message, target,
                 serial, by, channel, site, epoch="", uuid="", sensor_1="",
                 sensor_2=""):
        self.category = category
        self.package_type = package_type
        self.sensor_type = sensor_type
        self.message = message
        self.target = target
        self.serial = serial
        self.by = by
        self.channel = channel
        self.site = site
        self.epoch = epoch
        self.uuid = uuid
        self.sensor_1 = sensor_1
        self.sensor_2 = sensor_2
        self.validate_event()
        return
    
    def validate_event(self):
        if self.category == "":
            self.category = "unknown"
        if self.package_type == "":
            self.package_type = "unknown"
        if self.sensor_type == "":
            self.sensor_type = "unknown"
        if self.message == "":
            self.message = "unknown"
        if self.target == "":
            self.target = "unknown"
        if self.serial == "":
            self.serial = "unknown"
        if self.by == "":
            self.by = "unknown"
        if self.channel == "":
            self.channel = "unknown"
        if self.site == "":
            self.site = "unknown"
        if self.epoch == "":
            self.stamp = datetime.datetime.utcnow()
            self.epoch = str(time.time())
        else:
            self.stamp = datetime.datetime.utcfromtimestamp(float(self.epoch))
        if self.uuid == "":
            self.uuid = str(uuid.uuid4().hex)
        return
    
    def __str__(self):
        mystr = "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}"\
                    .format(str(self.stamp),
                            str(self.serial),
                            str(self.target),
                            str(self.message),
                            str(self.category),
                            str(self.by),
                            str(self.sensor_type),
                            str(self.package_type),
                            str(self.channel),
                            str(self.site))
        return mystr
    
    def get_json(self, secret=None, key=None):
        obj = {"channel":self.channel,
               "site":self.site,
               "secret":secret,
               "key":key,
               "action":self.action,
               "data":{"uuid":self.uuid,
                       "epoch":self.epoch,
                       "serial":self.serial,
                       "target":self.target,
                       "message":self.message,
                       "by":self.by,
                       "category":self.category,
                       "sensor_type":self.sensor_type,
                       "package_type":self.package_type}}
        if secret == None:
            del obj['secret']
        if key == None:
            del obj['key']
        return json.dumps(obj)
    
    def tag(self, created_by="", label="", value="", dataset="", experiment=""):
        if created_by == "":
            created_by = "unknown"
        if dataset == "":
            dataset = "unknown"
            raise ValueError("Tag.dataset must have an actual value!")
        if experiment == "":
            experiment = "unknown"
            raise ValueError("Tag.dataset must have an actual value!")
        t = Tag(self.category, self.package_type, self.sensor_type,
                self.message, self.target, self.serial, self.by, self.channel,
                self.site, self.epoch, self.uuid, created_by, label, value,
                dataset, experiment)
        return t



class Tag(Event):
    action = "tag"
    created_by = ""
    label = ""
    value = ""
    dataset = ""
    experiment = ""
    def __init__(self, category, package_type, sensor_type, message, target,
                 serial, by, channel, site, epoch, uuid, created_by, label,
                 value, dataset, experiment):
        self.category = category
        self.package_type = package_type
        self.sensor_type = sensor_type
        self.message = message
        self.target = target
        self.serial = serial
        self.by = by
        self.channel = channel
        self.site = site
        self.epoch = epoch
        self.uuid = uuid
        self.created_by = created_by
        self.label = label
        self.value = value
        self.dataset = dataset
        self.experiment = experiment
        self.validate_event()
        self.validate_tag()
        return
    
    def validate_tag(self):
        if self.created_by == "":
            self.created_by = "unknown"
        if self.dataset == "":
            self.dataset = "unknown"
            raise ValueError("Tag.dataset must have an actual value!")
        if self.experiment == "":
            self.experiment = "unknown"
            raise ValueError("Tag.experiment must have an actual value!")
        return
    
    def __str__(self):
        mystr = "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{} {}\t{}\t{}\t{}"\
                .format(str(self.stamp),
                        str(self.serial),
                        str(self.target),
                        str(self.message),
                        str(self.category),
                        str(self.by),
                        str(self.sensor_type),
                        str(self.package_type),
                        str(self.channel),
                        str(self.site),
                        str(self.label),
                        str(self.value),
                        str(self.created_by),
                        str(self.dataset),
                        str(self.experiment))
        return mystr
    
    def get_json(self, secret=None, key=None):
        obj = {"channel":self.channel,
               "site":self.site,
               "secret":secret,
               "key":key,
               "action":self.action,
               "data":{"tag":{"created_by":self.created_by,
                              "label":{"name":self.label,
                                       "value":self.value},
                              "dataset":self.dataset,
                              "experiment":self.experiment},
                       "uuid":self.uuid,
                       "epoch":self.epoch,
                       "serial":self.serial,
                       "target":self.target,
                       "message":self.message,
                       "by":self.by,
                       "category":self.category,
                       "sensor_type":self.sensor_type,
                       "package_type":self.package_type}}
        if secret == None:
            del obj['secret']
        if key == None:
            del obj['key']
        return json.dumps(obj)



class Control(object):
    action = "control"
    category = ""
    target = ""
    serial = ""
    by = ""
    channel = ""
    site = ""
    command = ""
    value = ""
    replyto = ""
    cid = ""
    response = ""
    package_type = ""
    sensor_type = ""
    message = ""
    stamp = ""
    stamp_local = ""
    epoch = ""
    uuid = ""
    sensor_1 = ""
    sensor_2 = ""
    
    def __init__(self, category, target, serial, by, channel, site, command,
                 value, replyto, cid, response="", package_type="control",
                 sensor_type="control", message="", epoch="", uuid="",
                 sensor_1="", sensor_2=""):
        self.category = category
        self.target = target
        self.serial = serial
        self.by = by
        self.channel = channel
        self.site = site
        self.command = command
        self.value = value
        self.replyto = replyto
        self.cid = cid
        self.response = response
        self.package_type = package_type
        self.sensor_type = sensor_type
        self.message = message
        self.epoch = epoch
        self.uuid = uuid
        self.validate_control()
        return
    
    def validate_control(self):
        if self.category == "":
            self.category = "control"
        if self.target == "":
            self.target = "unknown"
        if self.serial == "":
            self.serial = "unknown"
        if self.by == "":
            self.by = "unknown"
        if self.channel == "":
            self.channel = "unknown"
        if self.site == "":
            self.site = "unknown"
        if self.epoch == "":
            self.stamp = datetime.datetime.utcnow()
            self.epoch = str(time.time())
        else:
            self.stamp = datetime.datetime.utcfromtimestamp(float(self.epoch))
        if self.uuid == "":
            self.uuid = str(uuid.uuid4().hex)
        self.package_type = "control"
        self.sensor_type = "control"
        self.build_message()
        return
    
    def build_message(self):
        self.message = dict({'command':self.command,
                             'value':self.value,
                             'cid':self.cid,
                             'replyto':self.replyto})
        if self.response != "":
            self.message['response'] = self.response
        self.message = str(json.dumps(self.message, sort_keys=True))
        return
    
    def get_as_event_obj(self):
        self.build_message()
        e = Event(self.category, self.package_type, self.sensor_type,
                  self.message, self.target, self.serial, self.by,
                  self.channel, self.site, self.epoch, self.uuid)
        return e
    
    def __str__(self):
        mystr = "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}"\
                    .format(str(self.stamp),
                            str(self.serial),
                            str(self.target),
                            str(self.command),
                            str(self.value),
                            str(self.replyto),
                            str(self.cid),
                            str(self.by),
                            str(self.channel),
                            str(self.site))
        return mystr
    
    def get_json(self, secret=None, key=None):
        obj = {"channel":self.channel,
               "site":self.site,
               "secret":secret,
               "key":key,
               "action":self.action,
               "data":{"uuid":self.uuid,
                       "epoch":self.epoch,
                       "serial":self.serial,
                       "target":self.target,
                       "command":self.command,
                       "value":self.value,
                       "replyto":self.replyto,
                       "cid":self.cid,
                       "by":self.by,
                       "category":self.category}}
        if secret == None:
            del obj['secret']
        if key == None:
            del obj['key']
        return json.dumps(obj)
    
    def tag(self, created_by="", label="", value="", dataset="", experiment=""):
        e = self.get_as_event_obj()
        t = e.tag(created_by, label, value, dataset, experiment)
        return t





"""
[{"action": "event",
  "secret": "XXXX",
  "key": "XXXX",
  "data": {"category": "entity",
           "uuid": "6a3f05d6a4f243d5a76d5654d529db6a",
           "message": "ON",
           "package_type": "c4:cardaccess_inhome:WMS10-2",
           "epoch": "1445569748.405201",
           "sensor_type": "Control4-Motion",
           "serial": "000680000001150d",
           "by": "ZigbeeAgent",
           "target": "OfficeADesk"},
  "site": "tokyo",
  "channel": "rawevents"}]

[{"action": "tag",
  "secret": "XXXX",
  "key": "XXXX",
  "data": {"category": "entity",
           "uuid": "6a3f05d6a4f243d5a76d5654d529db6a",
           "message": "ON",
           "package_type": "c4:cardaccess_inhome:WMS10-2",
           "epoch": "1445569748.405201",
           "sensor_type": "Control4-Motion",
           "serial": "000680000001150d",
           "by": "ZigbeeAgent",
           "target": "M009",
           "tag": {"created_by": "RAT",
                   "label": {"name": "Work_On_Computer",
                             "value": ""},
                   "dataset": "P3302",
                   "experiment": "Test Experiment"}},
  "site": "tokyo",
  "channel": "tag"}]

[{"action": "control",
  "secret": "XXXX",
  "key": "XXXX",
  "data": {"category": "control",
           "uuid": "6a3f05d6a4f243d5a76d5654d529db6a",
           "command": "ON",
           "value": "",
           "replyto": "automate@tokyo",
           "cid": "42",
           "epoch": "1445569748.405201",
           "serial": "000680000001150d",
           "by": "AutomationAgent",
           "target": "LL009"},
  "site": "tokyo",
  "channel": "control"}]
"""
def build_objects_from_json(message):
    l.debug("build_objects_from_json( {} )".format(str(message)))
    response = dict({'status':'success', 'errormessage':'No Errors',
                         'type':'data', 'errors':list()})
    return_objects = list()
    try:
        blob = json.loads(message)
        if len(blob) == 0:
            response['status'] = 'error'
            response['errormessage'] = "The JSON array contains 0 values."
        element_id = 0
        for obj in blob:
            l.debug("object: {}".format(str(obj)))
            errormsgs = list()
            obj_uuid = "unknown"
            if 'action' not in obj:
                errormsgs.append("Could not obtain attribute action, please include json attribute action.")
            if 'site' not in obj:
                errormsgs.append("Could not obtain attribute site, please include json attribute site.")
            
            if len(errormsgs) == 0:
                if obj['action'] == 'event':
                    if 'channel' not in obj:
                        errormsgs.append("Could not obtain attribute channel, please include json attribute channel.")
                    if 'data' not in obj:
                        errormsgs.append("Could not obtain attribute data, please include json attribute data.")
                    else:
                        if 'category' not in obj['data']:
                            errormsgs.append("Could not obtain attribute category, please include json attribute data->category.")
                        if 'package_type' not in obj['data']:
                            errormsgs.append("Could not obtain attribute package_type, please include json attribute data->package_type.")
                        if 'epoch' not in obj['data']:
                            errormsgs.append("Could not obtain attribute epoch, please include json attribute data->epoch.")
                        if 'sensor_type' not in obj['data']:
                            errormsgs.append("Could not obtain attribute sensor_type, please include json attribute data->sensor_type.")
                        if 'message' not in obj['data']:
                            errormsgs.append("Could not obtain attribute message, please include json attribute data->message.")
                        if 'target' not in obj['data']:
                            errormsgs.append("Could not obtain attribute target, please include json attribute data->target.")
                        if 'serial' not in obj['data']:
                            errormsgs.append("Could not obtain attribute serial, please include json attribute data->serial.")
                        if 'by' not in obj['data']:
                            errormsgs.append("Could not obtain attribute by, please include json attribute data->by.")
                        if 'uuid' not in obj['data']:
                            errormsgs.append("Could not obtain attribute uuid, please include json attribute data->uuid.")
                        else:
                            obj_uuid = obj['data']['uuid']
                    
                    if len(errormsgs) == 0:
                        e = Event(obj['data']['category'],
                                  obj['data']['package_type'],
                                  obj['data']['sensor_type'],
                                  obj['data']['message'],
                                  obj['data']['target'],
                                  obj['data']['serial'],
                                  obj['data']['by'],
                                  obj['channel'],
                                  obj['site'],
                                  obj['data']['epoch'],
                                  obj['data']['uuid'])
                        return_objects.append(copy.deepcopy(e))
                        
                elif obj['action'] == 'tag':
                    if 'channel' not in obj:
                        errormsgs.append("Could not obtain attribute channel, please include json attribute channel.")
                    if 'data' not in obj:
                        errormsgs.append("Could not obtain attribute data, please include json attribute data.")
                    else:
                        if 'category' not in obj['data']:
                            errormsgs.append("Could not obtain attribute category, please include json attribute data->category.")
                        if 'package_type' not in obj['data']:
                            errormsgs.append("Could not obtain attribute package_type, please include json attribute data->package_type.")
                        if 'epoch' not in obj['data']:
                            errormsgs.append("Could not obtain attribute epoch, please include json attribute data->epoch.")
                        if 'sensor_type' not in obj['data']:
                            errormsgs.append("Could not obtain attribute sensor_type, please include json attribute data->sensor_type.")
                        if 'message' not in obj['data']:
                            errormsgs.append("Could not obtain attribute message, please include json attribute data->message.")
                        if 'target' not in obj['data']:
                            errormsgs.append("Could not obtain attribute target, please include json attribute data->target.")
                        if 'serial' not in obj['data']:
                            errormsgs.append("Could not obtain attribute serial, please include json attribute data->serial.")
                        if 'by' not in obj['data']:
                            errormsgs.append("Could not obtain attribute by, please include json attribute data->by.")
                        if 'uuid' not in obj['data']:
                            errormsgs.append("Could not obtain attribute uuid, please include json attribute data->uuid.")
                        else:
                            obj_uuid = obj['data']['uuid']
                        if 'tag' not in obj['data']:
                            errormsgs.append("Could not obtain attribute tag, please include json attribute data->tag.")
                        else:
                            if 'created_by' not in obj['data']['tag']:
                                errormsgs.append("Could not obtain attribute created_by, please include json attribute data->tag->created_by.")
                            if 'experiment' not in obj['data']['tag']:
                                errormsgs.append("Could not obtain attribute experiment, please include json attribute data->tag->experiment.")
                            if 'dataset' not in obj['data']['tag']:
                                errormsgs.append("Could not obtain attribute dataset, please include json attribute data->tag->dataset.")
                            if 'label' not in obj['data']['tag']:
                                errormsgs.append("Could not obtain attribute label, please include json attribute data->tag->label.")
                            else:
                                if 'name' not in obj['data']['tag']['label']:
                                    errormsgs.append("Could not obtain attribute name, please include json attribute data->tag->label->name.")
                                if 'value' not in obj['data']['tag']['label']:
                                    errormsgs.append("Could not obtain attribute value, please include json attribute data->tag->label->value.")
                    
                    if len(errormsgs) == 0:
                        t = Tag(obj['data']['category'],
                                obj['data']['package_type'],
                                obj['data']['sensor_type'],
                                obj['data']['message'],
                                obj['data']['target'],
                                obj['data']['serial'],
                                obj['data']['by'],
                                obj['channel'],
                                obj['site'],
                                obj['data']['epoch'],
                                obj['data']['uuid'],
                                obj['data']['tag']['created_by'],
                                obj['data']['tag']['label']['name'],
                                obj['data']['tag']['label']['value'],
                                obj['data']['tag']['dataset'],
                                obj['data']['tag']['experiment'])
                        return_objects.append(copy.deepcopy(t))
                        
                elif obj['action'] == 'control':
                    if 'channel' not in obj:
                        errormsgs.append("Could not obtain attribute channel, please include json attribute channel.")
                    if 'data' not in obj:
                        errormsgs.append("Could not obtain attribute data, please include json attribute data.")
                    else:
                        if 'command' not in obj['data']:
                            errormsgs.append("Could not obtain attribute command, please include json attribute data->command.")
                        if 'value' not in obj['data']:
                            errormsgs.append("Could not obtain attribute value, please include json attribute data->value.")
                        if 'replyto' not in obj['data']:
                            errormsgs.append("Could not obtain attribute replyto, please include json attribute data->replyto.")
                        if 'cid' not in obj['data']:
                            errormsgs.append("Could not obtain attribute cid, please include json attribute data->cid.")
                        if 'response' not in obj['data']:
                            obj['data']['response'] = ""
                        if 'category' not in obj['data']:
                            errormsgs.append("Could not obtain attribute category, please include json attribute data->category.")
                        if 'epoch' not in obj['data']:
                            errormsgs.append("Could not obtain attribute epoch, please include json attribute data->epoch.")
                        if 'target' not in obj['data']:
                            errormsgs.append("Could not obtain attribute target, please include json attribute data->target.")
                        if 'serial' not in obj['data']:
                            errormsgs.append("Could not obtain attribute serial, please include json attribute data->serial.")
                        if 'by' not in obj['data']:
                            errormsgs.append("Could not obtain attribute by, please include json attribute data->by.")
                        if 'uuid' not in obj['data']:
                            errormsgs.append("Could not obtain attribute uuid, please include json attribute data->uuid.")
                
                        obj['data']['package_type'] = "control"
                        obj['data']['sensor_type'] = "control"
                        
                        if len(errormsgs) == 0:
                            c = Control(obj['data']['category'],
                                        obj['data']['target'],
                                        obj['data']['serial'],
                                        obj['data']['by'],
                                        obj['channel'],
                                        obj['site'],
                                        obj['data']['command'],
                                        obj['data']['value'],
                                        obj['data']['replyto'],
                                        obj['data']['cid'],
                                        obj['data']['response'],
                                        obj['data']['package_type'],
                                        obj['data']['sensor_type'],
                                        "",
                                        obj['data']['epoch'],
                                        obj['data']['uuid'])
                            return_objects.append(copy.deepcopy(c))
                            
            if len(errormsgs) > 0:
                response['status'] = 'error'
                response['errormessage'] = "There were errors processing part or all of your event data, please see errors for details"
                response['errors'].append(dict({'element':element_id,
                                                'uuid':obj_uuid,
                                                'error':' | '.join(errormsgs)}))
                l.error("Error processing message: " + json.dumps(response['errors'][-1]))
            element_id += 1
    
    except TypeError as e:
        l.error("JSON TYPE ERROR: " + str(e))
        response['status'] = 'error'
        response['type'] = 'json'
        response['errormessage'] = "The conversion of JSON failed possibly due to improperly formatted JSON set in the message."
        response['errors'].append(str(e))
        l.debug(message)
        l.warning(json.dumps(response))
    except ValueError as e:
        l.error("JSON VALUE ERROR: " + str(e))
        response['status'] = 'error'
        response['type'] = 'json'
        response['errormessage'] = "The conversion of JSON failed possibly due to improperly formatted JSON set in the message."
        response['errors'].append(str(e))
        l.debug(message)
        l.warning(json.dumps(response))
    return return_objects

