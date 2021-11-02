# configure logging
#   set level to DEBUG

#   add format argument to basicConfig to display time::lineno::level::message
import logging

logFormat = '%(asctime)s::%(levelname)s::line %(lineno)d::%(message)s'

logging.basicConfig(level=logging.DEBUG, format=logFormat)

# will write all messages to console
logging.warning('oops, that may be a problem!')
logging.info('made it this far') 
logging.debug('the bug is here')


print("example 5 finished")
