## Simultaneous Localization and Mapping through Particle Filtering

An implementation of particle filtering algorithm for Simultaneous Localization and Mapping (SLAM)

Update: This was my assignment from grad school in 2007, I have not attempted to run it since and neither should you be attempting to run this code. I uploaded this to GitHub as it brough back pleasant old memories.

### Usage

```
python slam.py <datafile [ex:data/localized.data]> <cell size [ex:20]> <Type (-1/0/1/2,3) [ex:0]>"
```

### Demo

Here's a movie demonstrating SLAM in action on the file `data/localized.data`. This is a simulation of the robot receiving real-time data from its LASER range finder and SONAR and using SLAM to build a map of its surrounding and positioning itself in that environment.

![SLAM in Action](https://raw.githubusercontent.com/vatsan/slam/master/viz/slam_visualization.gif)
