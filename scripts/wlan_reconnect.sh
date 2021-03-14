#!/bin/bash
# Auto check if WiFi is connected. If not reconnect via DHCP
#
# Instructions:
#
# o Place file where you want it /usr/local/bin or leave it in ExoMy folder
# o chmod -x wlan_reconnect.sh
# o Add to crontab: sudo crontab -e
#
# Run Every 5 mins. If once a min change */5 to *
# If once every 2 mins change */5 to */2 ...
#
# */5 * * * * /usr/bin/sudo -H /home/pi/ExoMy_Software/scripts/wlan_reconnect.sh >> /dev/null 2>&1
#
# Change /etc/default/cron and add "EXTRA_OPTS="-L 0" to stop logging cronjob in syslog
# Restart cron: /etc/init.d/cron force-reload
#
##################################################################
# Settings
# Which Interface do you want to check/fix
wlan='wlan0'
# If IP to Ping is 0 use simple SSID fall back
iptoping='0'
##################################################################

echo "Starting WiFi check for $wlan"

if [ $iptoping != '0' ]
then
  echo "via Ping..."
  ping -c4 $iptoping > /dev/null

  if [ $? != 0 ] 
  then
    echo "No network connection, restarting $wlan"
    /sbin/dhcpcd -n $wlan
  else
    echo "... is functional"
  fi
else
  echo "via SSID..."
  iwconfig 2>&1 | grep ESSID > /dev/null

  if [ $? != 0 ] 
  then
    echo "No network connection, restarting $wlan"
    /sbin/dhcpcd -n $wlan
  else
    echo "... is functional"
  fi
fi

echo "Check and possible reconnect to $wlan complete!"