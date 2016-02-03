TurtleBot Aruco Code Follower
=============================

## Referenzen

Folgende Pakete wurden verwendet:

- [RPLIDAR driver wrapper](https://github.com/robopeak/rplidar_ros)
- [Aruco library wrapper](https://github.com/pal-robotics/aruco_ros)


## Anleitung:

## Anleitung für SLAM

#### Auf Roboter:

```
roscore
roslaunch goalfinder slam.launch
```

Mit den Teleop-Funktionen kann der Roboter nun bewegt werden, um eine Karte zu generieren.

#### Auf PC (nur für Visualisierung):

```
roslaunch goalfinder rviz_slam.launch
```

#### Karte speichern
Die Karte muss im Anschluss noch in der ```goalfinder/launch/\_amcl.launch``` eingebunden werden.
```
rosrun map_server map_saver -f mymap
```

## Anleitung für Follower Mode

#### Auf Roboter:

```
roscore
roslaunch goalfinder goalfinder.launch
```

#### Auf PC:

```
roslaunch goalfinder rviz_goals.launch
```

## Todo

- Marker in rviz mit textur darstellen, damit TF debug entfernt werden kann
- Path.msg entfernen

## Lizenz

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
