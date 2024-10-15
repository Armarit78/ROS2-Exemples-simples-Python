# ROS2 Exemples simple Overview

This README provides an overview and instructions for using the scripts in `my_package`. Each program is described below along with instructions on how to run them and associated commands.

## Table of Contents

- [1. pub.py](#1-pubpy)
- [2. verif.py](#2-verifpy)
- [3. pub_sql.py](#3-pubsqlpy)
- [4. sub_sql.py](#4-subsqlpy)
- [Quick Commands to Access a SQL Database](#quick-commands-to-access-a-sql-database)
- [5. pointcloud_translator.py](#5-pointcloudtranslatorpy)
- [6. pointcloud_comparator.py](#6-pointcloudcomparatorpy)
- [YAML Configuration and Publishing](#yaml-configuration-and-publishing)


## 1. pub.py

### Description
`pub.py` is a simple ROS2 publisher node that publishes random `Float32` values to the `sensor_topic`. This script can simulate a sensor's data output by periodically sending random floating-point values between 0 and 100.

### Code Overview
- Publishes random `Float32` data every second.
- Logs each published value.

### How to Run
```bash
source ~/ros2_ws/install/local_setup.bash
ros2 run my_package pub
```


## 2. verif.py

### Description

`verif.py` is a ROS2 node that subscribes to the word_topic, checking if the word "chat" was received, and publishes a response on response_topic. This is useful for basic word verification or responding to specific message data.

### Code Overview

    - Subscribes to word_topic to receive string messages.
    - Verifies if the message is "chat."
    - Publishes a response based on the verification result.

### How to Run

```bash
source ~/ros2_ws/install/local_setup.bash
ros2 run my_package verif
```

### Example Commands

Publish a word to the word_topic:

```bash
ros2 topic pub /word_topic std_msgs/String "data: 'chat'"
```


## 3. pub_sql.py

### Description

`pub_sql.py` subscribes to the sensor_topic and stores incoming Float32 data into a MySQL database. This node can be used to log sensor data continuously for later analysis or monitoring.

### Code Overview

    - Subscribes to sensor_topic to receive Float32 messages.
    - Connects to a MySQL database and inserts the received sensor data.
    - Logs data reception and database insertion events.

### How to Run

```bash
source ~/ros2_ws/install/local_setup.bash
ros2 run my_package pub_sql
```

### Prerequisites

    - Ensure MySQL server is running and accessible.
    - Configure MySQL user, password, and database name in the script.


## 4. sub_sql.py

### Description

`sub_sql.py` extracts the latest sensor data entries from a MySQL database every 5 seconds and publishes them to extracted_data_topic. It's ideal for extracting and reviewing the latest logged sensor data.

### Code Overview

    - Connects to a MySQL database to pull the latest data.
    - Publishes the latest data to extracted_data_topic.
    - Logs the data being published.

### How to Run

```bash
source ~/ros2_ws/install/local_setup.bash
ros2 run my_package sub_sql
```

### Prerequisites

    - Ensure MySQL server is running and accessible.
    - Database ros2_data with a table sensor_data must exist.


## Quick Commands to Access a SQL Database

Here are some quick commands for interacting with a SQL database using the MySQL command-line client:

Access MySQL Console

```bash
mysql -u root -p
```
Replace root with your MySQL username. You will be prompted to enter your MySQL password.

#### Show Databases

```sql
SHOW DATABASES;
```

#### Select a Database

```sql
USE ros2_data;
```

#### Show Tables in the Database

```sql
SHOW TABLES;
```

#### Query Data from a Table

```sql
SELECT * FROM sensor_data;
```

These commands will help you manually verify the database structure and content, ensuring your ROS2 nodes are interacting with the database as expected. Adjust user credentials and database names according to your configuration.


## 5. pointcloud_translator.py(PointCloud Translator)

### Description

`pointcloud_translator.py` is a ROS2 node that subscribes to a point cloud topic (perf/points), translates all points by a fixed offset, and publishes the translated point cloud to a new topic (translated_pointcloud). This program is used to manipulate and modify point cloud data dynamically.

### Code Overview

    - Subscribes to the point cloud data.
    - Applies a translation to each point (x, y, z) by adding 2.0 to their current values.
    - Publishes the translated point cloud data.

### How to Run

```bash
source ~/ros2_ws/install/local_setup.bash
ros2 run my_package pointcloud_translator
```

### Prerequisites

    - Make sure to have the sensor_msgs package and appropriate Python bindings for message and serialization handling.


## 6. pointcloud_comparator.py (PointCloud Comparator)

### Description

`pointcloud_comparator.py` is a ROS2 node designed to compare two point cloud data sets: original and translated. It subscribes to both the original (perf/points) and translated (translated_pointcloud) point cloud topics, compares them, and prints the differences in the console.

### Code Overview

    - Subscribes to both original and translated point cloud topics.
    - Stores and compares point data from both topics.
    - Logs the comparison results, highlighting the transformations applied to each point.

### How to Run

```bash
source ~/ros2_ws/install/local_setup.bash
ros2 run my_package pointcloud_comparator
```

### Prerequisites

Ensure both pointcloud_translator.py and pointcloud_comparator.py are running simultaneously for real-time comparison.


## YAML Configuration and Publishing

For programs that utilize YAML configurations, such as the ones that handle point cloud data, you can publish data using the YAML file through the command line:

```bash
source install/local_setup.bash
cat test.yaml | xargs -0 ros2 topic pub /perf/points sensor_msgs/msg/PointCloud2
```
