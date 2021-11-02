# configure logging
#   set level to DEBUG
#   write messages to file, no logging messages will display in the console
#   specify write mode "w" so that previous version of file is overwritten.
import logging

logging.basicConfig(filename='example4.log', filemode='w', level=logging.DEBUG)

# will write all messages to file 
logging.warning('oops, that may be a problem!')
logging.info('made it this far') 
logging.debug('the bug is here')


print("example 4 finished")
