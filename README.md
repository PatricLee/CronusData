# CronusData
This repository processes Velodyne data as described in my Darknet-Velodyne repository: https://github.com/PatricLee/Darknet, which is my thesis, where I use Velodyne data and CNN to detect surrounding vehicles.

## 0 Requirements

- **MS VS 2015**, or v14.0
- **OpenCV 3.0**

## 1 Compile

When you meet the requirements mentioned above, open `CronusData.sln`, make sure OpenCV is correctly included, set project to be **Debug** and **x64**, compile!

I can't really think of any problem you would have... If you have any problem that you think you can't solve, issue/direct contact is welcomed.

## 2 Set Paths and Parameters

Now if you digged into the code even a little bit, you'd find that NO, THERE'S NO INTERFACE, **LITERALLY**.

So how do you set parameters? How do you even set the input and output paths?

### 2.1 Paths
Open `CronusData/CronusData.cpp`, will you? Locate two variables: 'dir' and 'dir_out', that's the input and output paths.

I use Kitti dataset for my thesis, after downloading there are two folders: 'training' and 'testing'. Since I only have labels for training set, I used only 'training'. Under 'training' folder there should be at least two folders: 'velodyne' and 'label'. This path, `.../training/`, is your input path.

Your output path is a folder you created that is writeable. Take my output path for example, `D:/Kitti/`, then the program puts the generated pictures and labels under folder `D:/Kitti/velo/`, and training and validating lists, lists with absolute paths of every picture, will be under `D:/Kitti/`. You **DO** have to create `.../velo/` folder before you run the program or it would lead to unexpected behaviour.

### 2.2 Training List Size
Look at the top lines of `CronusData.cpp`, you'd find a macro 'TRAINNUM', this determines how many pictures will there be in the training set, and the rest will be validating set. since Kitti training set has something around 7500 frames of velodyne data, I set this macro to be 4000.

Note that Kitti has already randomized its training set, so I didn't randomize anything and just took first 4000 pictures as the training set.

### 2.3 Parameters of the Picture
Open `CronusData/lidar_to_image.hpp`, here're the functions and macros used to convert velodyne data into pictures I used.

If you've checked out my Darknet-Velodyne repository you know I converted velodyne data into pictures in bird view, or ground grid.
- The size of the grid is determined by macros 'PicWidth' and 'PicHeight' in pixel, which are both 304 in my case.
- The actual area in bird view in velodyne coordinate is determined by macros 'PicTop'(front), 'PicDown'(back), 'PicLeft'(left) and 'PicRight'(right), in meters. in my case it's a 30.4x30.4m area in front of the vehicle.
- Height is determined by 'HeightMax' and 'HeightMin' in meters. Points exceeding this range in height will be ignored.
- 'MinLabelSize' is a threshold that object smaller than this threshold on the image shall be ignored. This one follows Kitti setting, which is 50.

### 2.4 Class
In `lidar_to_image.hpp`, locate a function called 'GetObjectClass'. This funciton returns the object class in integer, if it's -1 then it won't be printed. So since I detect only vehicles, I set 'Car', 'Van' and 'Truck' to be 0, and others to be -1.

## 3 Run

Now that you've done doing all that stuff, you may compile again and run the program. This might take a while.

When running, it would show in a window the currently genenrated picture and label, note that those boxes would not show on the saved picture.
