# configure logging level, set to INFO level
# prints to console by default
import logging

logging.basicConfig(level=logging.INFO)

# will print both warning and info messages
logging.warning('oops, that may be a problem!')
logging.info('made it this far') 

# will not print debug, because debug is lower level than info
logging.debug('the bug is here')
