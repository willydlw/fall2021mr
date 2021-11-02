# Python Logging

Logging is a means of tracking events that happen when some software runs. The software’s developer adds logging calls to their code to indicate that certain events have occurred. An event is described by a descriptive message which can optionally contain variable data (i.e. data that is potentially different for each occurrence of the event). Events also have an importance which the developer ascribes to the event; the importance can also be called the *level* or *severity*.

Programmers often use print messages when running a python script to provide meaningful messages and values stored in variables. For small programs, this is often all you need for development. This approach does not scale well to larger projects with multiple modules. The python logging library provides a more flexible approach.

Using logging, you can:

Control message level to log only required ones
Control where to show or save the logs
Control how to format the logs with built-in message templates
Know which module the messages is coming from



### Logging Levels

| Level | When it’s used |
| --- | --- |
| DEBUG | Detailed information, typically of interest only when diagnosing problems. |
| INFO | Confirmation that things are working as expected. |
| WARNING | An indication that something unexpected happened, or indicative of some problem in the near future (e.g. ‘disk space low’). The software is still working as expected. |
| ERROR | Due to a more serious problem, the software has not been able to perform some function. |
| CRITICAL | A serious error, indicating that the program itself may be unable to continue running. |

The default level is WARNING, which means that only events of this level and above will be tracked, unless the logging package is configured to do otherwise.

Events that are tracked can be handled in different ways. The simplest way of handling tracked events is to print them to the console. Another common way is to write them to a disk file.



## Example 1 - Simple Logging Example

The logging library contains functions that match the level. The example below has one message for the warning level and one for the info level. 

```
import logging
logging.warning('oops, that may be a problem!')  # will print a message to the console
logging.info('made it this far')  # will not print anything
```

Run the script and observe the console output: `WARNING:root:oops, that may be a problem!`

The INFO message doesn’t appear because the default level is WARNING. The next example will use the config function to set the level. Note that the printed message includes the indication of the level. Ignore the 'root' part for now.


## Example 2 - Configuring Logging Level

The script below calls the config file to set the logging level. With the level set to INFO, both the warning and info messages will print to the console. The debug message will not print because it is a lower level than INFO.

```
import logging
logging.basicConfig(level=logging.INFO)
logging.warning('oops, that may be a problem!')
logging.info('made it this far') 
logging.debug('the bug is here')
```


basicConfig must be called before calling debug(), info(), etc. Additional calls to basicConfig will have no effect.


## Example 3 - Configuring Logging to a File, Append Mode

The default configuration is console, but you may also configure logging message to a file. Add a filename name parameter to the script, as shown below, and run it. There will be no console output. Both messages will be written to the file.

```
import logging
logging.basicConfig(filename='example.log', level=logging.INFO)
logging.warning('oops, that may be a problem!')
logging.info('made it this far') 
```


## Example 4 - Configuring Logging to a File, Overwrite Mode

Each time this script is run, messages will be appended to the log file. If you want to overwrite the file contents, instead of appending, specify the file mode argument.

`logging.basicConfig(filename='example.log', filemode='w', level=logging.DEBUG)`


## Example 5 - Log Record Attributes

Example 5 illustrates specifying a format for logging messages.

```
logFormat = '%(asctime)s::%(levelname)s::line %(lineno)d::%(message)s'
logging.basicConfig(level=logging.DEBUG, format=logFormat)
```

https://docs.python.org/3/library/logging.html#logging.Formatter 


## Example 6 - Logging Variable Data

Use the specifiers %s for string, %d for integer, %f for floating point.

```
msg = 'hello'
count = 5
temperature = 12.1 

logging.warning('temperature: %f', temperature) 
logging.debug('count: %d, temperature: %.2f', count, temperature)
logging.info('msg: %s', msg)
```





## Reference

More information may be found at https://docs.python.org/3/howto/logging.html

