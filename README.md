# Air-Thief---CanSat

The aim of our project is proving that microbes exist at high altitudes  - more specifically 1-2 km into our troposphere. We want to build a satellite that is able to collect air samples from high altitudes. These samples would be then filtered in order to get rid of any contaminants leaving only the desired microbes in between the filters. Additionally, the results of our experiment, apart from promoting the idea that life is possible on other planets than Earth, could help understand better the life cycle of bacteria here on Earth. Painting a big picture of growth and needs of simple microbiological organisms could help us understand how to fight the harmful ones and how to effectively make use of neutral or beneficial organisms.

One of our main objectives is to make project an open-source - this means we want all of our work to be available online to potentially benefit similar projects in the future. Moreover, it makes following our project easier than ever. Below you will be able to view and download all of our projects. This is the purpose of our GitHub page.

# Table of Contents

* [Introduction](#introduction)
* [CanSat description](#CanSat-description)
*[Mission Overview](#mission-overview)
* [Hardware Design](#hardware-design)
 **[3D Models](#3d-models)
 ** [Bill of Materials](#bill-of-materials)
* [Electrical Design](#electrical-design)
* [How to code the Air Thief](#how-to-code-the-air-thief)
*[Ground support equipment](#ground-support-system)
  ** [License](#license)

# Introduction
The Air Thief team consists of 5 members from Akademeia High School. We formed a team as we believed the CanSat competition is a one-time experience that will allow us to polish our skills as well as gain newfound skills from the process. We came up with the idea of creating the Air Thief - a satellite that would be able to collect air from an altitude of 2km, which can be later sampled for microbes. Throughout the competition, we have brainstormed how we will achieve this, as well as have gained partners such as Adamed, Cloudferro, JLCPCB, CubicInch and Thorium Space Technology, who see great potential in our work and are willing to support us with their resources.

The medium through which we want to accomplish our mission is the CanSat competition - a competition organized by the European Space Agency with the goal of building a real satellite within the volume of a soft drink can. This competition takes place every year and is held to very high standards as initially, over 80 teams per country participate in the challenge.

There are many opportunities for patenting ideas, meaning this is definitely an area for business. As an example, the recent mission exploring life on Venus, which would require a satellite that would be able to collect microorganisms from the atmosphere.

# How to program the Air Thief

The primary onboard computing unit for our CanSat is an Adafruit M0 that supports Arduino. It is going to run all the programs necessary for the functioning of the CanSat and all onboard equipment and experiments. The flight plan for individual atmospheric Microbiome soundings can be fine-tuned, which is helpful. The data will be recorded to a MicroSD card, which will probably have such high storage capacity that it will be virtually infinite for our purposes (16 GB, around 30 zloty).
The program has 3 main modes, controlled based on the current altitude measured by the temperature-pressure sensor. This is going to ensure the correct data is always transmitted and minimum power is consumed. If the AdaFruit sensor detects an anomalous result, the mode will have a 3000 millis switch cooldown so that it is not turned on preemptively. Our reasoning is that since the primary mission is so important, relying on it for data is quite sensible.

[CanOS for Air Thief](https://github.com/aleksychwedczuk/Air-Thief---CanSat/blob/main/Software/CanOS.ino)

## Modes:
 ### Standby – When the satellite is waiting on the launchpad and when it has landed post-experiment are similar flight conditions and require a similar approach. Thus, Standby mode is active when the CanSat elevation reported via the AdaFruit array is less than 100 meters AGL. During this mode, the CanSat is only running the first experiment, sampling the ambient temperature and pressure, and computing the altitude at regular 10 second intervals, ready to switch into an active state when the altitude goes beyond 100 meters. The buzzer is on while in this mode only if the Sampling flag has been set, meaning that the vessel has entered sampling altitude at least once. The buzzer pulses at an optimal frequency that can be heard from a large distance (2700 Hz)
 ### Active – When the satellite is in flight, above 100 meters AGL, it constantly calculates its position and AGL via the GPS and AdaFruit sensors. [This means sampling occurs at 500 millis intervals.] These parameters are then transmitted to the ground station, so that a flight profile can be determined. If the altitude were to increase or decrease, Sampling or Standby modes would be engaged, respectively.
 ### Sampling – The pump powering the secondary experiment is enabled. It runs constantly until either the total runtime requirement is satisfied (so as not to overshoot) or until the CanSat goes below 80% of the mission altitude (2 km for a 2.5 km mission, for instance, this is configured pre-launch). That way the sample is collected from the correct experimental band that was being sampled (for instance one that has a height of 0.5 km). During Sampling, the Active activities are also conducted. If the sampling altitude is passed and the CanSat begins heading down again, Active, and then finally Standby modes are engaged. When Sampling mode is entered for the first time [since last boot], a system-wide flag is set that is later used to determine whether the Buzzer should be turned on while in Standby.
The data recorded via the MicroSD card, and the outgoing transmissions sent out via radio will have a specific format that will minimize their size, enabling higher efficiency. [This means that the timestamps used for instance are going to be epoch time that can be later converted into human-readable time values, not in situ in the POCU but in the gstat after receiving it.] The ground station program will be coded in JS for the frontend, and in Python3 for the backend.

# Hardware Resources

## Electrical design

Our CanSat is made up of a couple specific components, each has its one role in the mission. The components chosen here are finalized by the Critical Design Review. The main controller board is the brain of the operation; it is used for measuring the primary and controlling the secondary mission. The processor used will be a Feather M0 module that involves an ARM Cortex M0 microcontroller known from Arduino Zero with an integrated radio module (LoRa) for communication purposes.
The core of our primary mission will be an MPL temperature and pressure sensor, which will record the data every second to give us an accurate depiction of the altitude of the CanSat. This will be calculated using the hypsometric formula, more information in section 7.1: Test Campaign: Primary mission tests. This sensor suits our mission well because of its high level of accuracy. The core of our secondary mission will be the NW Air Pump which will be used to push air from a high altitude through a series of filters. This air pump after testing showed that the amount of air that it can pump is sufficient for finding small organisms and bacteria. Furthermore, the power converter that was initially used for powering the pump was not strong enough and it needed to be changed for a one that could supply more power. To power the NW Air Pump with 6V and 400 mA current a step-down converter (D24V10F6) will be used to change the battery voltage of 11.1 V, also a second step down converter (D24V22F5) will be used to power the main controller with 5V and 700 mA.  The converter powering the pump was changed after testing due to the current limit being reached (500mA) and the motor not reaching its highest power possible. To power the CanSat three Li-Ion 750 mAh batteries are used and the voltage is stepped down for the other components. To turn on the pump a relay is used for safety in case of a short in the motor the motherboard will not be damaged. It is particularly important for our secondary mission that the CanSat will be found due to the physical sample that we need to analyze in a lab; therefore, a GPS module is used to determine the coordinates of the CanSat, and these will be sent to the ground station. To help with finding the CanSat after landing an 80dB buzzer is used, if the CanSat is hard to find visibly it can be also found using sound.

![CanSat Mappedout](https://github.com/aleksychwedczuk/Air-Thief---CanSat/blob/main/cansatmappedout2.png)

# Ground support equipment
Changes in CDR: The POCU handles only basic data processing and categorizing. Additionally, there is more data sent out via Radio than is saved to the SD card (since ICAR pings and other health information are only saved on the ground station). The POCU handles only basic data processing, handling & categorizing. Additionally, there is more data sent out via Radio than is saved to the SD card (since the ICAR pings and other health information are only saved on the ground station).
The ground station is composed of these elements:
 	Laptop running the backend and frontend for the communication with the satellite. The frontend handles the display of data on screen and the issuing of commands to the CanSat, and the backend communicates directly and writes to files, etc. The frontend is written in JS. The backend works in Python. [One of our programmers proposed an additional backend (a middle end if You will) that will kick in if some gstat OS crash occurs, to prevent any local data loss. This is just called a glorified log file, but still.]
 	A YAGI omnidirectional antenna that will send and receive data from the CanSat. It is connected to the Laptop.
 	A power supply for the laptop and the antenna. This is a backup, as there is most likely mains power in situ at the launch site.
 	A port-a-nanolab with materials applicable for the final selected procedure [if needed, consult detailed procedure report for fully written-out action plan etc.]:
1.	Lab safety equipment (coat, glasses, gloves)
2.	Flow cytometer in Adamed’s Laboratory (letter of intent signed and link available below)
3.	Cytometry prep kit
4.	Professional glove box
5.	Self-adhesive foil
6.	Isorapid spray or ethanol
7.	Set of tools to open the chambers
8.	Tip or inoculating loop or cotton swabs
9.	UVC lamp (inside or outside)
10.	Sterile swabs
11.	Sterile agar plates
12.	Transportation box
13.	Plastic zipper bags
 	Alternative support equipment that we considered to conduct the analysis of the microbes (provided by Adamed) that might be used as a backup form of conducting the analysis [updated]:
1.	UV-C filter
2.	Standard amateur-grade light microscope [has role of backup & validation device, capable of detecting microbes within light diffraction limit, that means it is easily capable of detecting bacteria]
3.	Immersion oil for oil immersion of microscope objective lens (sic!)
4.	PCR Thermocycler with master mix, primers, etc. for Virus DNA amplification
5.	Electrophoresis equipment for DNA electrophoresis post-PCR
6.	SYBR Green Dye dispenser, plus other dyes and chemicals needed for the procedure.
7.	Viral genome extraction equipment
8.	UV-Vis spectrophotometer for virus quantification
9.	Lab-grade Fluorescent Microscope
10.	Accurate scale and other basic measurement equipment
11.	Sars-CoV-2 Quick-Antibody detection kit for testing teammates (pandemic control related if needed).
12.	Microorganisms instant detection kit (obsolete).

 


## 3D Models

The following files are uploaded in the 3D folder:
