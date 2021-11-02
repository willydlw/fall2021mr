# configure logging
#   set level to INFO
#   write messages to file, no logging messages will display in the console
#      run the script more than once. Will append output to the file.
import logging

logging.basicConfig(filename='example3.log', level=logging.INFO)

# will write both warning and info messages to file 
logging.warning('oops, that may be a problem!')
logging.info('made it this far') 

# will not print debug, because debug is lower level than info
logging.debug('the bug is here')


print("example 3 finished")
