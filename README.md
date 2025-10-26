🛰️ LoRa-Based Localisation System for Drones in GPS-Denied Environments

This project was developed as part of the RMIT University Engineering Capstone (2024–2025) in collaboration with Fusion Engineering Consulting Services, a defence contractor.
It focuses on creating a low-cost, long-range localisation system for drones operating in environments where GPS is unavailable or unreliable.

🚀 Project Overview

The system utilises LoRa (Long Range) radio communication to estimate the position of a moving drone based on Received Signal Strength Indicator (RSSI) measurements from multiple ground anchors.
By combining statistical filtering and trilateration algorithms, the system can produce stable and repeatable position estimates in both stationary and dynamic conditions.

🧠 Key Features

LoRa Communication Network:
Configured multiple LoRa anchors and a mobile node for real-time RSSI data collection.

RSSI Filtering Techniques:
Implemented Median Filter and Exponential Moving Average (EMA) to reduce noise and improve signal stability.

Trilateration Algorithm:
Developed MATLAB-based localisation logic to estimate drone position from filtered signal strengths.

Data Analysis & Visualisation:
Generated plots showing raw vs filtered RSSI data, variance reduction, and positional accuracy improvements.



Public Demonstration:
Presented at EnGenius 2025 (Melbourne Exhibition Centre) showcasing real-time filtering and localisation outputs.

🧩 Technologies Used

MATLAB – Signal filtering, data analysis, and visualisation

Arduino / LoRa Modules – Wireless communication and data collection

Excel – Statistical comparison and result tabulation

Fusion Engineering Consulting Services – Defence collaboration partner
