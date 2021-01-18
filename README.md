# Air-Thief---CanSat

The aim of our project is proving that microbes exist at high altitudes  - more specifically 1-2 km into our troposphere. We want to build a satellite that is able to collect air samples from high altitudes. These samples would be then filtered in order to get rid of any contaminants leaving only the desired microbes in between the filters. Additionally, the results of our experiment, apart from promoting the idea that life is possible on other planets than Earth, could help understand better the life cycle of bacteria here on Earth. Painting a big picture of growth and needs of simple microbiological organisms could help us understand how to fight the harmful ones and how to effectively make use of neutral or beneficial organisms.

One of our main objectives is to make project an open-source - this means we want all of our work to be available online to potentially benefit similar projects in the future. Moreover, it makes following our project easier than ever. Below you will be able to view and download all of our projects. This is the purpose of our GitHub page.

# Table of Contents

* [Introduction](#introduction)
* [Hardware Specification](#hardware-specification)
* [Inside the Air Thief]
* [Getting started with the Air Thief Cansat]
* [How to program the Air Thief]
* [Hardware Resources](#hardware-resources)
  * [3D Models](#3d-models)
  * [Bill of Materials](#bill-of-materials)
* [License](#license)
# Introduction
The Air Thief team consists of 5 members from Akademeia High School. We formed a team as we believed the CanSat competition is a one-time experience that will allow us to polish our skills as well as gain newfound skills from the process. We came up with the idea of creating the Air Thief - a satellite that would be able to collect air from an altitude of 2km, which can be later sampled for microbes. Throughout the competition, we have brainstormed how we will achieve this, as well as have gained partners such as Adamed, Cloudferro, JLCPCB, CubicInch and Thorium Space Technology, who see great potential in our work and are willing to support us with their resources.

The medium through which we want to accomplish our mission is the CanSat competition - a competition organized by the European Space Agency with the goal of building a real satellite within the volume of a soft drink can. This competition takes place every year and is held to very high standards as initially, over 80 teams per country participate in the challenge.

There are many opportunities for patenting ideas, meaning this is definitely an area for business. As an example, the recent mission exploring life on Venus, which would require a satellite that would be able to collect microorganisms from the atmosphere.

# Hardware Specification

The hardware of the Air Thief CanSat consists mostly of elements that can be either purchased at your local hardware store or can be printed using a 3D printer. The following product tree describes all components required for construction of each subsystem, and lists all items that go into each of them. There is also a diagram that helps you understand what to connect to what

# How to program the Air Thief

The primary onboard computing unit for our CanSat is an Adafruit M0 that supports Arduino. It is going to run all the programs necessary for the functioning of the CanSat and all onboard equipment and experiments. The flight plan for individual atmospheric Microbiome soundings can be fine-tuned, which is helpful. The data will be recorded to a MicroSD card, which will probably have such high storage capacity that it will be virtually infinite for our purposes (16 GB, around 30 zloty).
The program has 3 main modes, controlled based on the current altitude measured by the temperature-pressure sensor. This is going to ensure the correct data is always transmitted and minimum power is consumed. If the AdaFruit sensor detects an anomalous result, the mode will have a 3000 millis switch cooldown so that it is not turned on preemptively. Our reasoning is that since the primary mission is so important, relying on it for data is quite sensible.
## Modes:
 ### Standby – When the satellite is waiting on the launchpad and when it has landed post-experiment are similar flight conditions and require a similar approach. Thus, Standby mode is active when the CanSat elevation reported via the AdaFruit array is less than 100 meters AGL. During this mode, the CanSat is only running the first experiment, sampling the ambient temperature and pressure, and computing the altitude at regular 10 second intervals, ready to switch into an active state when the altitude goes beyond 100 meters. The buzzer is on while in this mode only if the Sampling flag has been set, meaning that the vessel has entered sampling altitude at least once. The buzzer pulses at an optimal frequency that can be heard from a large distance (2700 Hz)
 ### Active – When the satellite is in flight, above 100 meters AGL, it constantly calculates its position and AGL via the GPS and AdaFruit sensors. [This means sampling occurs at 500 millis intervals.] These parameters are then transmitted to the ground station, so that a flight profile can be determined. If the altitude were to increase or decrease, Sampling or Standby modes would be engaged, respectively.
 ### Sampling – The pump powering the secondary experiment is enabled. It runs constantly until either the total runtime requirement is satisfied (so as not to overshoot) or until the CanSat goes below 80% of the mission altitude (2 km for a 2.5 km mission, for instance, this is configured pre-launch). That way the sample is collected from the correct experimental band that was being sampled (for instance one that has a height of 0.5 km). During Sampling, the Active activities are also conducted. If the sampling altitude is passed and the CanSat begins heading down again, Active, and then finally Standby modes are engaged. When Sampling mode is entered for the first time [since last boot], a system-wide flag is set that is later used to determine whether the Buzzer should be turned on while in Standby.
The data recorded via the MicroSD card, and the outgoing transmissions sent out via radio will have a specific format that will minimize their size, enabling higher efficiency. [This means that the timestamps used for instance are going to be epoch time that can be later converted into human-readable time values, not in situ in the POCU but in the gstat after receiving it.] The ground station program will be coded in JS for the frontend, and in Python3 for the backend.

# Hardware Resources

## 3D Models

The following files are uploaded in the 3D folder:
