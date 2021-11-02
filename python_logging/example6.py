# log variable data
import logging

logFormat = '%(asctime)s::%(levelname)s::line %(lineno)d::%(message)s'

logging.basicConfig(level=logging.DEBUG, format=logFormat)

# create some example variables
msg = 'hello'
count = 5
temperature = 12.1 

# will write all messages to console
logging.warning('temperature: %f', temperature) 
logging.debug('count: %d, temperature: %.2f', count, temperature)
logging.info('msg: %s', msg)


print("example 6 finished")
