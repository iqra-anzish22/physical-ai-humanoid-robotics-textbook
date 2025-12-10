---
title: ROS 2 سیٹ اپ اور کنفیگریشن
sidebar_position: 2
description: روبوٹکس ترقی کے لیے ROS 2 انسٹال کرنے اور کنفیگر کرنے کے بارے میں مکمل گائیڈ
keywords: [ros2, installation, setup, configuration, robotics]
learning_outcomes:
  - مختلف آپریٹنگ سسٹم پر ROS 2 انسٹال کرنا
  - ROS 2 ورک سپیس اور ماحول کنفیگر کرنا
  - ROS 2 پیکجز تخلیق اور تعمیر کرنا
  - ROS 2 تقسیم اور ورژننگ کو سمجھنا
---

import LearningOutcome from '@site/src/components/LearningOutcome/LearningOutcome';

# ROS 2 سیٹ اپ اور کنفیگریشن

<LearningOutcome outcomes={[
  "مختلف آپریٹنگ سسٹم پر ROS 2 انسٹال کرنا",
  "ROS 2 ورک سپیس اور ماحول کنفیگر کرنا",
  "ROS 2 پیکجز تخلیق اور تعمیر کرنا",
  "ROS 2 تقسیم اور ورژننگ کو سمجھنا"
]} />

## تعارف

ROS 2 کو سیٹ کرنا روبوٹکس ایپلیکیشنز ترقی دینے کا پہلا قدم ہے۔ یہ گائیڈ آپ کو مختلف پلیٹ فارمز پر انسٹالیشن کے عمل اور ROS 2 کے ساتھ ترقی شروع کرنے کے لیے ضروری بنیادی کنفیگریشن کے ذریعے لے جائے گا۔

## ROS 2 تقسیمات

ROS 2 لینکس تقسیمات کی طرح تقسیم کا ماڈل استعمال کرتا ہے، جہاں ہر تقسیم کا ایک نام ہوتا ہے (مثلاً فوکسی، گیلیکٹک، ہمبل، آئرن) اور ایک تائید کا وقت ہوتا ہے۔ موجودہ تجویز کردہ تقسیم **ہمبل ہاکسبل** ہے، جو ایک LTS (لمبی مدت کی تائید) ریلیز ہے جس کی تائید 2027 تک ہوگی۔

### تقسیم کا ٹائم لائن

- **ہمبل ہاکسبل (humble)**: LTS ریلیز، 2027 تک تائید
- **آئرن ایروینی (iron)**: معیاری ریلیز، نومبر 2024 تک تائید
- **رولنگ رڈلی (rolling)**: ترقیاتی ریلیز، جاری طور پر اپ ڈیٹ ہوتی رہتی ہے

تیاری اور سیکھنے کے مقاصد کے لیے، ہم ہمبل ہاکسبل استعمال کرنے کی تجویز کرتے ہیں۔

## اوبنٹو لینکس پر انسٹالیشن

### سسٹم کی ضروریات

- اوبنٹو 22.04 (جمی جیلی فش) - تجویز کردہ
- کم از کم 5GB مفت ڈسک کی جگہ
- پیکجز ڈاؤن لوڈ کرنے کے لیے انٹرنیٹ کنکشن

### انسٹالیشن اقدامات

1. **مقام کو سیٹ کریں**
   ```bash
   locale-gen en_US.UTF-8
   export LANG=en_US.UTF-8
   ```

2. **ماخذ کو سیٹ کریں**
   ```bash
   sudo apt update && sudo apt install curl gnupg lsb-release
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   ```

3. **ROS 2 ذخیرہ کو شامل کریں**
   ```bash
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

4. **ROS 2 پیکجز انسٹال کریں**
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

5. **colcon تعمیر کے ٹولز انسٹال کریں**
   ```bash
   sudo apt install python3-colcon-common-extensions
   ```

6. **ROS ترقیاتی ٹولز انسٹال کریں** (اختیاری لیکن تجویز کردہ)
   ```bash
   sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
   sudo rosdep init
   rosdep update
   ```

## ونڈوز پر انسٹالیشن

### سسٹم کی ضروریات

- ونڈوز 10 (ورژن 1909 یا بعد کا) یا ونڈوز 11
- WSL2 (ونڈوز سبسٹم برائے لینکس) کے ساتھ اوبنٹو 22.04 تجویز کردہ
- کم از کم 5GB مفت ڈسک کی جگہ

### انسٹالیشن اقدامات

1. **WSL2 کے ساتھ اوبنٹو 22.04 انسٹال کریں**
   ```powershell
   wsl --install -d Ubuntu-22.04
   ```

2. **WSL2 ٹرمنل کے اندر اوبنٹو انسٹالیشن کے اقدامات کو فالو کریں**

3. **GUI ایپلیکیشنز کے لیے X11 فارورڈنگ کنفیگر کریں** (اختیاری)
   - VcXsrv یا X410 جیسے X-سرور انسٹال کریں
   - DISPLAY ماحولیاتی متغیر سیٹ کریں

## میک او ایس پر انسٹالیشن

### سسٹم کی ضروریات

- میک او ایس 12 (مونٹیری) یا بعد کا
- ہومبرو پیکج مینیجر
- کم از کم 5GB مفت ڈسک کی جگہ

### انسٹالیشن اقدامات

1. **ہومبرو انسٹال کریں** (اگر پہلے سے انسٹال نہ ہو)
   ```bash
   /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
   ```

2. **بائنری پیکجز کا استعمال کرتے ہوئے ROS 2 انسٹال کریں**
   ```bash
   # انحصار انسٹال کریں
   brew install python@3.10
   brew install asio cppcheck eigen3 log4cxx opencv pcre poco tinyxml2 uncrustify
   brew install graphviz
   pip3 install -c https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos ros2cli
   ```

## ماحول کا سیٹ اپ

### ROS 2 ماحول کو سورس کرنا

انسٹالیشن کے بعد، آپ کو ROS 2 کا ماحول ہر ٹرمنل میں سورس کرنا ہوگا جہاں آپ ROS 2 استعمال کرنا چاہتے ہیں:

```bash
source /opt/ros/humble/setup.bash
```

اسے مستقل بنانے کے لیے، مندرجہ ذیل لائن کو اپنی `~/.bashrc` فائل میں شامل کریں:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### ROS 2 ورک سپیس تخلیق کرنا

1. **ورک سپیس ڈائرکٹری تخلیق کریں**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   ```

2. **ورک سپیس تعمیر کریں**
   ```bash
   colcon build
   ```

3. **ورک سپیس کو سورس کریں**
   ```bash
   source install/setup.bash
   ```

## پیکج مینجمنٹ

### ایک نیا پیکج تخلیق کرنا

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --dependencies rclcpp rclpy std_msgs my_robot_package
```

### پیکجز تعمیر کرنا

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_package
```

## ROS 2 ماحولیاتی متغیرات

ROS 2 کے لیے کلیدی ماحولیاتی متغیرات:

- `ROS_DOMAIN_ID`: ROS ڈومین ID سیٹ کرتا ہے (ڈیفالٹ: 0)
- `RMW_IMPLEMENTATION`: مڈل ویئر نافذ کاری کی وضاحت کرتا ہے
- `ROS_LOG_DIR`: ROS لاگ فائلز کے لیے ڈائرکٹری
- `ROS_LOCALHOST_ONLY`: صرف لوکل ہوسٹ کے ذریعے رابطہ کو زور دیتا ہے (1) یا نہیں (0)

## تصدیق

اپنی انسٹالیشن کی تصدیق کرنے کے لیے، ٹاکر/لسنر ڈیمو چلانے کی کوشش کریں:

```bash
# ٹرمنل 1
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker

# ٹرمنل 2
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

اگر آپ دیکھتے ہیں کہ ٹاکر اور لسنر کے درمیان پیغامات بھیجے جا رہے ہیں، تو آپ کی انسٹالیشن کامیاب ہے!

## مسئلہ حل کرنا

### عام مسائل

1. **پیکج نہیں ملا**: یقین دہانی کرائیں کہ ROS کمانڈز استعمال کرنے سے پہلے ماحول کو سورس کریں
2. **اجازت کی خرابیاں**: فائل کی اجازتوں کی جانچ کریں اور یقین دہانی کرائیں کہ آپ صحیح صارف استعمال کر رہے ہیں
3. **نیٹ ورک کے مسائل**: ROS 2 DDS استعمال کرتا ہے جس کو مناسب نیٹ ورک کنفیگریشن کی ضرورت ہوتی ہے
4. **پائی تھون پاتھ کے تنازعات**: یقین دہانی کرائیں کہ پائی تھون پیکجز صحیح ماحول میں انسٹال ہیں

## اگلے اقدامات

اب جب آپ کے پاس ROS 2 انسٹال اور کنفیگر ہے، آپ ROS 2 کے تصورات کو تلاش کر سکتے ہیں اور اپنی پہلی روبوٹکس ایپلیکیشنز تخلیق کر سکتے ہیں۔ اگلے سیکشن عملی مثالیں اور مشقیں کو کور کرے گا تاکہ آپ کی سمجھ کو مضبوط کیا جا سکے۔

## حوالہ جات

1. ROS 2 Documentation. (2023). Installation Guide. Retrieved from https://docs.ros.org/en/humble/Installation.html
2. Open Robotics. (2023). ROS 2 User Guide. ROS Documentation.
3. Quigley, M., Gerkey, B., & Smart, W. (2022). Programming Robots with ROS: A Practical Introduction to the Robot Operating System. O'Reilly Media.