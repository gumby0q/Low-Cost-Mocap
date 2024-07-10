
## My notes
<!-- activate -->
```sh
source my-env/bin/activate
cd computer_code
python3 api/index.py
```

```sh
cd computer_code
yarn run dev
```



### Script to allow access to camera without `sudo`
```sh
computer_code/cam_rules.sh
```

### ESP code
ArduinoJson
https://github.com/bblanchon/ArduinoJson
https://arduinojson.org/v7/how-to/install-arduinojson/


PID v1 install as zip
https://github.com/br3ttb/Arduino-PID-Library/tree/master?tab=readme-ov-file

sbus
https://github.com/bolderflight/sbus

## python things


instal it into python `venv`?
https://docs.python.org/3/tutorial/venv.html
```sh
python -m venv my-env
```
install dependences
```sh
sudo apt-get install python3-tk
pip install Pillow
pip install h5py
pip install flask
pip install flask-socketio
pip install serial # dk if that needed???
pip install ruckig
pip install flask_cors
pip install pyserial

pip install matplotlib
pip install pandas
```

### camera libs code, pseye, fixes
patched pseye https://github.com/Tintin-Axelsson/pseyepy/tree/master

https://github.com/Tintin-Axelsson/pseyepy/tree/master

### openCV
can be useful?
https://github.com/alyssaq/reconstruction

https://stackoverflow.com/a/45582705

!!!!

? https://docs.opencv.org/4.x/d2/de6/tutorial_py_setup_in_ubuntu.html

? https://gist.github.com/alvaro893/6125aa58f8a1bb3895f75684af0ea24d

full guide (but without tunes)
https://bksp.space/blog/en/2020-03-01-compiling-opencv-with-structure-from-motion-module-python-bindings.html

#### this is the way
```sh
source /path/to/your/venv/bin/activate
```

```sh
git clone <opencv>
git clone <opencv_contrib>
cd <opencv folder>
mkdir buid
cd build
```
```sh
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/home/anatolii/projects/Low-Cost-Mocap/my-env \
    -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
    -DBUILD_opencv_sfm=ON \
    -DBUILD_opencv_python_bindings_generator=ON \
    -DPYTHON_EXECUTABLE=/home/anatolii/projects/Low-Cost-Mocap/my-env/bin/python \
    -DPYTHON_INCLUDE_DIR=/home/anatolii/projects/Low-Cost-Mocap/my-env/include/python3.10 \
    -DPYTHON_LIBRARY=/home/anatolii/projects/Low-Cost-Mocap/my-env/lib/libpython3.10.so \
    -DINSTALL_PYTHON_EXAMPLES=ON \
    -DBUILD_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF

```
```
make -j8
```
```sh
sudo make install
```

### Other things

some mocap system
https://docs.ipisoft.com/User_Guide_for_Multiple_PS_Eye_Cameras_Configuration

### Camera, lens

MINI-2,8 M12 F=2,8 мм, угол обзора 81x65°, F 2,0 1/3" Подробнее: https://tehbezpeka.ua/p108997168-obektiv-dlya-kamer.html
Фиксированный 3-х мегапиксельный объектив для охранных камер видеонаблюдения f=2,8 мм, угол обзора 135 ° (мод. LS-2.8M) – 1 шт. Подробнее: https://tehbezpeka.ua/p108997168-obektiv-dlya-kamer.html

https://wavelength-oe.com/uk/optical-calculators/field-of-view/
Focal Length (mm)*
2.8
Pixel Size (μm)*
12 (not sure, 6um when resolution twice(forth) bigger?)
Detector Horizontal Pixels*
320
Detector Vertical Pixels*
240
Horizontal FOV (°)
68.9
Vertical FOV (°)
54.4
Diagonal FOV (°)
81.2
Image Size (mm)
4.8

other calc https://www.scantips.com/lights/fieldofview.html#top

epipolar geometry
https://www.youtube.com/watch?v=cLeF-KNHgwU&list=PLgnQpQtFTOGQh_J16IMwDlji18SWQ2PZ6&index=36

ps3 eye camera
https://www.playstation.com/content/dam/global_pdc/en/corporate/support/manuals/accessories/ps3-accessories/sceh-00448-ps-eye/SCEH-00448_PS%20Eye_IM%24en.pdf
320x240

intrinsic-matrix
https://stackoverflow.com/questions/78072261/how-to-find-cameras-intrinsic-matrix-from-focal-length

https://stackoverflow.com/questions/25874196/camera-calibration-intrinsic-matrix-what-do-the-values-represent


https://www.reddit.com/r/astrophotography/comments/k0bieg/the_moon_2411_with_a_ps3_eye_camera_8_dobsonian/
 Since it had a 6 um pixelsize, based on a chart I saw and my focal length of 1200

https://astrobeano.blogspot.com/2013/03/ps3-eye-camera-for-astrophotography.html 
http://image-sensors-world.blogspot.com/2010/10/omnivision-vga-sensor-inside-sony-eye.html

https://html.alldatasheet.com/html-pdf/312422/OMNIVISION/OV7725/599/1/OV7725.html

some mocap software
https://docs.ipisoft.com/User_Guide_for_Multiple_PS_Eye_Cameras_Configuration


### Camera calibration

#### info
https://www.youtube.com/watch?v=GUbWsXU1mac


https://www.youtube.com/watch?v=O-7TkOKeK2M

https://github.com/tizianofiorenzani/how_do_drones_work/blob/master/opencv/ChessBoard_9x6.jpg


guy explains how to calibrate camera and where to get matrix and distortion coefs
https://youtu.be/EWqqseIjVqM?si=Wy8BY6NYLw00Lx1P&t=241

camera calibration
https://github.com/jyjblrd/Low-Cost-Mocap/discussions/23#discussioncomment-8118097

#### Camera calibration scripts 
based on

https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html

https://learnopencv.com/camera-calibration-using-opencv/

use next chessboard image
![alt text](images/ChessBoard_6x5.png)

create folders
```sh
cd computer_code
mkdir cam_1
mkdir cam_1_c
```

run from `Low-Cost-Mocap/computer_code/` to get images from pseye camera
```sh
python api/camera_pseyepy_collect_images.py
```

run to calc camera params from collected images 
```sh
python  api/camera_calc_params.py
```
remove bad recognized images from `cam_1` if you got such and run it again!!!


```js
// my 3 cams
// self.cameras = Camera(fps=90, ids=[2, 1, 0], resolution=Camera.RES_SMALL, gain=10, exposure=100)
[{"R":[[1,0,0],[0,1,0],[0,0,1]],"t":[0,0,0]},{"R":[[0.1722637774422725,0.7827356242616863,-0.5980385719106981],[-0.7672480350097106,0.48737412426679616,0.4168895726315438],[0.6177828451446811,0.3870289466006923,0.6845092773206958]],"t":[0.7170268722889785,-0.46208281725881506,0.35540442147238177]},{"R":[[-0.9822094024646937,-0.13421744172852396,0.13134065648457616],[0.160581676447473,-0.2377053740843171,0.9579716490172107],[-0.09735612409739951,0.9620196637633168,0.25502931524326106]],"t":[-0.20578254670461457,-1.0456168396141954,0.8016965557594956]}]


[[0.9941338485260931,0.0986512964608827,-0.04433748889242502,0.9938296704767513],[-0.0986512964608827,0.659022672138982,-0.7456252673517598,2.593331619023365],[0.04433748889242498,-0.7456252673517594,-0.6648888236128887,2.9576262456228286],[0,0,0,1]]

// with origin
[{"R":[[1,0,0],[0,1,0],[0,0,1]],"t":[0,0,0]},{"R":[[0.1722637774422725,0.7827356242616863,-0.5980385719106981],[-0.7672480350097106,0.48737412426679616,0.4168895726315438],[0.6177828451446811,0.3870289466006923,0.6845092773206958]],"t":[0.7170268722889785,-0.46208281725881506,0.35540442147238177]},{"R":[[-0.9822094024646937,-0.13421744172852396,0.13134065648457616],[0.160581676447473,-0.2377053740843171,0.9579716490172107],[-0.09735612409739951,0.9620196637633168,0.25502931524326106]],"t":[-0.20578254670461457,-1.0456168396141954,0.8016965557594956]}]

[[0.9941338485260931,0.0986512964608827,-0.04433748889242502,-0.0983113022924278],[-0.0986512964608827,0.659022672138982,-0.7456252673517598,1.2642083707956262],[0.04433748889242498,-0.7456252673517594,-0.6648888236128887,0.5346970677406504],[0,0,0,1]]

```



# Low Cost Mocap (for drones)

### A general purpose motion capture system built from the ground up, used to autonomously fly multiple drones indoors

## Dependencies
Install the pseyepy python library: [https://github.com/bensondaled/pseyepy](https://github.com/bensondaled/pseyepy)

This project requires the sfm (structure from motion) OpenCV module, which requires you to compile OpenCV from source. This is a bit of a pain, but these links should help you get started: [SFM dependencies](https://docs.opencv.org/4.x/db/db8/tutorial_sfm_installation.html) [OpenCV module installation guide](https://github.com/opencv/opencv_contrib/blob/master/README.md)

`cv.sfm` is only used 3 times in the codebase for the following functions: `fundamentalFromProjections`, `essentialFromFundamental`, `motionFromEssential`. So really, those functions should just be reimplemented in Python so the sfm module isn't needed. [Issue](https://github.com/jyjblrd/Mocap-Drones/issues/4).

install npm and yarn

## Runing the code

From the computer_code directory Run `yarn install` to install node dependencies 

Then run `yarn run dev` to start the webserver. You will be given a url view the frontend interface.

In another terminal window, run `python3 api/index.py` to start the backend server. This is what receives the camera streams and does motion capture computations.

## Documentation
The documentation for this project is admittedly pretty lacking, if anyone would like to put type definitions in the Python code that would be amazing and probably go a long way to helping the readability of the code. Feel free to also use the [discussion](https://github.com/jyjblrd/Mocap-Drones/discussions) tab to ask questions.

My blog post has some more information about the drones & camera: [joshuabird.com/blog/post/mocap-drones](https://joshuabird.com/blog/post/mocap-drones)

## YouTube Video
Watch this for information about the project & a demo!
[https://youtu.be/0ql20JKrscQ?si=jkxyOe-iCG7fa5th](https://youtu.be/0ql20JKrscQ?si=jkxyOe-iCG7fa5th)
![](https://github.com/jyjblrd/Mocap-Drones/blob/main/images/thumbnail.png?raw=true)

## Architectural Diagram
![](https://github.com/jyjblrd/Mocap-Drones/blob/main/images/architecture.png?raw=true)



DRONE XYZ: -0.421  0.154  0.296 heading:-0.141559525860047

відтестити спочатку ось z а потім вже інші

# PID
- insite of corrections https://support.haltech.com/portal/en/kb/articles/pid 

- to read
https://www.motor.com/magazine-summary/lean-codes-january-2018/

https://blog.isa.org/avoid-common-tuning-mistakes-pid



cam 4
--
Camera matrix : 

[[298.99006289   0.         161.32109398]
 [  0.         299.42961614 110.6700218 ]
 [  0.           0.           1.        ]]
dist : 

[[-0.44986403  0.23210829 -0.00055899 -0.0012163  -0.04040831]]

--
Camera matrix : 

[[296.08070343   0.         157.84371972]
 [  0.         296.86045377 110.13393665]
 [  0.           0.           1.        ]]
dist : 

[[-4.53996849e-01  2.43709559e-01 -4.58199466e-04  1.49375448e-04
  -5.20374383e-02]]

--
Camera matrix : 

[[298.78153724   0.         155.65271248]
 [  0.         299.44206683 109.72329526]
 [  0.           0.           1.        ]]
dist : 

[[-0.46417394  0.2875456  -0.0015743   0.00275365 -0.10989633]]

-------------------------------------------------------------------
cam5

--
Camera matrix : 

[[292.10898560888006, 0.0, 153.89221850831544], [0.0, 292.6438099161577, 113.75136764872889], [0.0, 0.0, 1.0]]
dist : 

[[-0.4608811901376308, 0.22994720931299295, -0.0005331331019399099, -0.000405784211482536, 0.023207746448878176]]

--
Camera matrix : 

[[291.22213989741357, 0.0, 153.07756297841155], [0.0, 291.89402656615306, 114.2228524239505], [0.0, 0.0, 1.0]]
dist : 

[[-0.4701866085336506, 0.3198007742424406, -0.0012036331463641642, 0.0016700905042756284, -0.15262477294797025]]

--
Camera matrix : 

[[292.703270277679, 0.0, 146.61972264237917], [0.0, 293.1574236183727, 111.17827266026785], [0.0, 0.0, 1.0]]
dist : 

[[-0.5114745208936426, 0.7083102669592334, 0.0006815406559609269, 0.006487398300484308, -1.1633417171021625]]


-------------------------------------------------------------------
cam6

--
Camera matrix : 

[[297.62849756147006, 0.0, 149.8868375195354], [0.0, 298.09425352323206, 119.41178244373978], [0.0, 0.0, 1.0]]
dist : 

[[-0.4618826043899917, 0.3659245527579734, 0.00026534337288099883, -0.0006843775663061602, -0.5074394706549016]]

--
Camera matrix : 

[[293.1590173489139, 0.0, 145.91082169090075], [0.0, 293.4749803472345, 117.1717862219495], [0.0, 0.0, 1.0]]
dist : 

[[-0.46850092213098793, 0.2972538029912042, 0.0014339490233973538, 0.0029808835480361778, -0.12054326832123326]]

--
Camera matrix : 

[[294.26456692343703, 0.0, 146.77932662237308], [0.0, 294.762127378571, 118.52558657500316], [0.0, 0.0, 1.0]]
dist : 

[[-0.47948813276044894, 0.3391955078691854, 0.0003754219875396906, 0.001140600199984154, -0.16758270665442973]]



OFFICE
CALLIB 1



----
[{"R":[[1,0,0],[0,1,0],[0,0,1]],"t":[0,0,0]},{"R":[[-0.732965757761099,0.507135908918787,-0.453403096410703],[-0.5263839533038304,-0.0006248859168639098,0.8502467543141939],[0.4309073353447679,0.861865870888682,0.2674062245785708]],"t":[0.31809713048900023,-0.9157224481501407,0.7062132993297783]},{"R":[[-0.7988753059613473,-0.3691664712731604,0.47488352468040684],[0.5176081780622515,-0.019757901115365836,0.8553896184468175],[-0.3063984652837548,0.929153239154467,0.20686768389606297]],"t":[-0.21559112348429524,-0.8993824171679524,0.7267776404545474]}]

[[0.9941338485260931,0.0986512964608827,-0.04433748889242502,-0.0983113022924278],[-0.0986512964608827,0.659022672138982,-0.7456252673517598,1.2642083707956262],[0.04433748889242498,-0.7456252673517594,-0.6648888236128887,0.5346970677406504],[0,0,0,1]]

---

[{"R":[[1,0,0],[0,1,0],[0,0,1]],"t":[0,0,0]},{"R":[[-0.7481377276961004,0.5845627037828804,-0.31396876555441583],[-0.48183774077491415,-0.15328502273572292,0.8627491485767085],[0.45620426561206323,0.7967371882565524,0.39634268113136734]],"t":[0.2918213767577657,-0.8416736685307064,0.6749755805937093]},{"R":[[-0.7151452998708677,-0.6223552089856887,0.31818421381486867],[0.55145065660319,-0.22264445173449685,0.8039475240609647],[-0.42949897951510546,0.7504021868039995,0.5024213218358253]],"t":[-0.3250996874039088,-0.856529323614061,0.6062485278518934]}]


[[0.9941338485260931,0.0986512964608827,-0.04433748889242502,-0.0983113022924278],[-0.0986512964608827,0.659022672138982,-0.7456252673517598,1.2642083707956262],[0.04433748889242498,-0.7456252673517594,-0.6648888236128887,0.5346970677406504],[0,0,0,1]]



---
[{"R":[[1,0,0],[0,1,0],[0,0,1]],"t":[0,0,0]},{"R":[[-0.7598609030604723,0.5704636915622835,-0.3117412141653001],[-0.4726296002640217,-0.15552109359317318,0.8674297956617899],[0.44635486885929077,0.8064641133098952,0.3877924251308809]],"t":[0.29742317264933743,-0.8561803750050299,0.7713094928645554]},{"R":[[-0.7093794883314872,-0.6461751011396129,0.2814933040087613],[0.5590245040718858,-0.27257704876025535,0.783066635948903],[-0.42926954863085726,0.712853064211946,0.5545882828386294]],"t":[-0.3425278761988839,-0.891032022959365,0.6256766755664085]}]


[[0.9941338485260931,0.0986512964608827,-0.04433748889242502,0.9938296704767513],[-0.0986512964608827,0.659022672138982,-0.7456252673517598,2.593331619023365],[0.04433748889242498,-0.7456252673517594,-0.6648888236128887,2.9576262456228286],[0,0,0,1]]





/default
[{ "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "t": [0, 0, 0] }, { "R": [[-0.0008290000610233772, -0.7947131755287576, 0.6069845808584402], [0.7624444396180684, 0.3922492478955913, 0.5146056781855716], [-0.6470531579819294, 0.46321862674804054, 0.6055994671226776]], "t": [-2.6049886186449047, -2.173986915510569, 0.7303458563542193] }, { "R": [[-0.9985541623963866, -0.028079891357569067, -0.045837806036037466], [-0.043210651917521686, -0.08793122558361385, 0.9951888962042462], [-0.03197537054848707, 0.995730696156702, 0.0865907408997996]], "t": [0.8953888630067902, -3.4302652822708373, 3.70967106300893] }, { "R": [[-0.4499864100408215, 0.6855400696798954, -0.5723172578577878], [-0.7145273934510732, 0.10804105689305427, 0.6912146801345055], [0.5356891214002657, 0.7199735709654319, 0.4412201517663212]], "t": [2.50141072072536, -2.313616767292231, 1.8529907514099284] }]

[[0.9941338485260931, 0.0986512964608827, -0.04433748889242502, 0.9938296704767513], [-0.0986512964608827, 0.659022672138982, -0.7456252673517598, 2.593331619023365], [0.04433748889242498, -0.7456252673517594, -0.6648888236128887, 2.9576262456228286], [0, 0, 0, 1]]



----

[{"R":[[1,0,0],[0,1,0],[0,0,1]],"t":[0,0,0]},{"R":[[-0.7598609030604723,0.5704636915622835,-0.3117412141653001],[-0.4726296002640217,-0.15552109359317318,0.8674297956617899],[0.44635486885929077,0.8064641133098952,0.3877924251308809]],"t":[0.4375795775020436,-1.2596430984950588,1.1347780302542596]},{"R":[[-0.7093794883314872,-0.6461751011396129,0.2814933040087613],[0.5590245040718858,-0.27257704876025535,0.783066635948903],[-0.42926954863085726,0.712853064211946,0.5545882828386294]],"t":[-0.5039392257660184,-1.3109180857506355,0.9205178362817863]}]


[[0.9941338485260931,0.0986512964608827,-0.04433748889242502,0.27131847221237565],[-0.0986512964608827,0.659022672138982,-0.7456252673517598,0.7464307711932481],[0.04433748889242498,-0.7456252673517594,-0.6648888236128887,0.5685358268244227],[0,0,0,1]]

<!--  -->
[[0.9941338485260931,0.0986512964608827,-0.04433748889242502,0.42606430244737287],[-0.0986512964608827,0.659022672138982,-0.7456252673517598,2.0502662121651167],[0.04433748889242498,-0.7456252673517594,-0.6648888236128887,1.0990447124346436],[0,0,0,1]]