#!/usr/bin/python
# -*- coding:utf-8 -*-
import logging, sys, traceback

class glog_py:
    NDEBUG = True           #enable debug_mode by setting NDEBUG = False
    glog_level=[
        #logging.DEBUG,     #debug_mode
        logging.INFO,       #0
        logging.WARNING,    #1
        logging.ERROR,      #2
        logging.CRITICAL]   #3
    log_level = logging.INFO
    logger = -1

    def __init__(self, level, name):
        if not self.NDEBUG:
            self.log_level = logging.DEBUG
        else:
            self.log_level = self.glog_level[level]
        self.logger = logging.getLogger(name)
        self.logger.setLevel(self.log_level)
        ch = logging.StreamHandler(sys.stderr)
        fmt = "%(levelname).1s%(asctime)s.%(msecs)03d    %(process)d %(filename)s:%(lineno)d] %(message)s"
        datefmt = "%m%d %H:%M:%S"
        formatter = logging.Formatter(fmt, datefmt)
        ch.setFormatter(formatter)
        self.logger.addHandler(ch)
        self.logger.warn("logger [%s] created, level: %s", self.logger.name, logging.getLevelName(self.log_level))

    def get_glogger(self):
        return self.logger

def get_glogger(level, name):
    glog = glog_py(level, name)
    return glog.get_glogger()

if __name__ == '__main__':
    level = 0
    glogger = get_glogger(level, __file__)
    
    msg = "this is glog style logging module! \n"
    debug = logging.DEBUG
    info = logging.INFO
    warn = logging.WARNING
    error = logging.ERROR
    critical = logging.CRITICAL

    glogger.info(msg)
    glogger.debug("replace \"rospy.logdebug\" with \"glogger.debug\": %d", debug)
    glogger.info("replace \"rospy.loginfo\" with \"glogger.info\": %d", info)
    glogger.warn("replace \"rospy.logwarn\" with \"glogger.warn\": %d", warn)
    glogger.error("replace \"rospy.logerr\" with \"glogger.error\": %d", error)
    glogger.critical("replace \"rospy.logfatal\" with \"glogger.critical\": %d", critical)
