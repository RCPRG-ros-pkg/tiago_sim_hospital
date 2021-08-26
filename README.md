# tiago_sim_hospital

Repozytorium symulacji robota Tiago w środowisku szpitalnym, wykorzystujące pakiet [tiago_sim_integration](https://github.com/RCPRG-ros-pkg/tiago_sim_integration).

# Zawartość

## Świat i modele

Świat gazebo tego repozytorium znajduje się w folderze
```
/worlds
```
Modele nowych mebli wykorzystywane w świecie są w folderze
```
/models
```
Oryginalne modele stworzone w programie Blender znajdują się w katalogu
```
/blender_ws
```

## Używanie

Do mapowania świata zawartego w paczce należy uruchomić launch:
```
/launch/tiago_mapping_hospital_v1.launch
```
Następnie w środowisku rviz należy zmienić opcję **Global Options -> Fixed Frame** z **map** na **odom**. Po świecie można poruszać się z wykorzystaniem narzędzia **2D Nav Goal**. Alternatywnie można do tego celu wykorzystać udostępniany przez pal pakiet **key_teleop**, pozwalający na kontrolowanie robota strzałkami klawiatury. W tym celu należy w oddzielnej konsoli uruchomić komendę:
```
rosrun key_teleop key_teleop.py
```
Stworzoną przez gmapping mapę można zapisać uruchamiając komendę:
```
rosrun map_server map_saver -f <target_directory/map_name>
```