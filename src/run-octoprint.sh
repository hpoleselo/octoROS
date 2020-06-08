output=$(ls /dev/ | grep USB)
# Tests if the output from the command above is empty or not.
if test -z "$output"
then
  echo "Trying ttyACM0..."
  sudo chmod 666 /dev/ttyACM0
  octoprint serve
else
  echo "Opening port for ttyUSB0"
  sudo chmod 666 /dev/ttyUSB0
  octoprint serve
fi