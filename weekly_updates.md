# Weekly Updates:

All students are required to send a weekly report via email describing,  briefly, the progress made during that period. It should include:

- **A list of supervision meetings** that have occurred since the last update
- A brief description of **progress on actions** from the last update
- **Other progress** on the project since the last update
- **Objectives** to complete before the next update - these should be measurable, e.g.
  - Order specific components (so should be ordered by next update)
  - Install software package (so should be installed by next update)
  - Update literature review (typically an ongoing task but some progress should be measurable and reported)
- **Comments** on any other aspects of the project
  - E.g. supervision, facilities, work environment, outside interest in the project
- Details of any **concerns** regarding the project

The report should be sent via email to Adar (who will respond with feedback) and copied to Jihong & [engineering-project-updates@york.ac.uk](mailto:engineering-project-updates@york.ac.uk)

# 23rd February

**Supervision Meetings:**

- No supervision meetings

**Progress on Actions:**

- First update, no previous objectives

**Other Progress:**

- Visited LIVE lab on Monday to have a look at EEG
  - Jack informed me that it was incorrectly setup so we didn't manage to do anything with it
  - Revisiting on Wednesday when Ellie should be available to show us how to use it
- Just received a USB drive for OS installation

**Objectives for Next Update:**

- Partition drive on laptop, install Ubuntu and ROS
- Create project code repository for version control and backup purposes
- Create visual stimuli for collecting EEG data
  - Will use movie.py for this to create configurable test videos
- Work on initial report

**Questions I have:**

- Should I be setting up weekly meetings with Jihong?
  - Is there guidance for the contents of these meetings?
- I realise the LIVE lab is usually locked - who should I ask if I want to access the lab? 
- Are there rules pertaining to linking a Git repository to GitHub
  - Is this allowed at all
  - If allowed, does it for example have to be set to private or can it be public?


# 1st March

**Supervision Meetings:**

- Meeting with Jihong 29/02
  - Discussed many of the points bought up in this weekly update

**Progress on Actions:**

- Installed Ubuntu 20.04.6 LTS and ROS1 on laptop*
- Created Git repository
  - Available on Github @ https://github.com/smithjjoseph/EEG-robotic-hand/tree/master

- Created a VEP type test stimulus (available via GitHub)
  - An alternating chequered pattern for testing visual-neural pathways
  - Created using SVG so should be usable on any web browser
- Progress made on initial report

**Objectives for Next Update:**

- Meet with Jack to learn how to collect some data from the EEG
- Create more test videos for EEG testing
  - Video with instruction to do action with hand
- Meet with Jihong at ISA to test laptop ROS setup
- Finish initial report

Comments:

- These weekly updates are also available via the Git repository

**Questions/Concerns:**

I have already asked Jihong the following however we decided second opinions would be useful for the first 2 questions

- On documents should I use the full original name of the project: 'Robotic Hand Control using EEGs/EMGs' or am I free to instead use an adjusted name: 'Robotic Hand Control using EEG'?
- Am I required to fill in an ethics form for this project? I do have a human subject (myself)
- On the PPR I was given feedback that I was notably missing a timetable/Gantt chart however in the brief for the assessment it explicitly said not to include a detailed project plan such as a Gantt Chart as it is a requirement for the initial report.
  - Did this discrepancy have an effect on my marks for this?




*Laptop was already setup with Windows. Wanted to dual-boot. Shrunk Windows partition. Disabled secure boot. Tried Ubuntu installation. Notified that storage controller needed changing (Intel RST to AHCI). Needed to make registry edits and change storage controller in BIOS. Windows expectedly BSODed. Boot files were corrupted so they had to be remade from recovery files. Tried all solutions on Ubuntu website. Tried some of my own solutions. Windows recovery partition also corrupted making it unusable without new installation media. Ended up formatting drive and just installing Ubuntu on a fresh drive.

# 8th March

**Supervision Meetings:**

- Met with Jihong on 08/03
  - Tested simple ROS implementation on laptop

**Progress on Actions:**

- Created test video specific to hand movement
  - Uses shapes that are associated with actions relating to hand movement
  - Lightly based on https://www.frontiersin.org/articles/10.3389/fnhum.2022.901285/full
- I had previously asked Jack to tell me when he was next able to show me the EEG system setup in the LIVE lab however I have not heard back from him since
  - I will reprompt him via email to tell me when he will next be in the lab just in case he forgot
- ROS is working on my laptop and we were able to get the simple opening and closing of the hand to work
- Initial report finished

**Other Progress:**

- Added central dot to VEP-like test video
  - I found that my eyes would wonder if I didn't put something specific to look at in the middle of the screen

**Objectives for Next Update:**

- Hopefully visit the LIVE lab and have Jack show me the EEG system
- Research ROS implementation and attempt to create incremental movement program for softhand

**Questions/Concerns:**

- No questions

# 15th March

**Supervision Meetings:**

- No supervision meeting with Jihong this week since Jihong is doing a talk and we couldn't find another time

**Progress on Actions:**

- Re-emailed Jack and now have session planned for Monday so he can show me how to use the EEG headset
- Didn't get a chance to work on the script for the incremental movement of the softhand - will continue this work for next week

**Objectives for Next Update:**

- Visit the LIVE lab on Monday and have Jack show me the EEG system
- Attempt implementation of incremental movement program for softhand

**Comments**

- Haven't had the chance to do as much work as I would've liked this week - I hope to get a lot more done next week

# 22nd March

**Supervision Meetings:**

- Meeting over Zoom with Jihong on 21/03

  - Showed and explained to him my video-tests

  - Discussed potential issues with operating the EEG Cap

    - Physical hardware is temperamental 

    - Possibly don't have the software (for Linux drivers)
    - Decided that if we can't get the EEG cap working I will instead base hand movement off computer vision (likely if EEG cap not sorted over next 2 weeks)

  - Discussed demonstration day logistics 
    - Need to notify department of location for the day
    - softhand can be dismounted and transported to West campus on demo day

**Progress on Actions:**

- Visited LIVE lab with Jack so he could show me how to use the system
  - We could not seem to get the EEG cap working when we met at the lab - there seemed to possibly be an issue where we didn't have the correct software
  - Ellie has since sent the drivers and software required however it is only for Windows and I only have a Linux laptop
    - Could run a VM but this would not allow for serial making this solution infeasable
  - I have asked Jack if there are any Linux drivers for the EEG system but I have not yet had a response so I will likely email Ellie as well
- Annotated simple hand open and close script

**Other Progress:**

- Installed OpenViBE on my laptop for experimentation purposes
  - Depending on the output of the EEG system this may or may not be the best way to pre-process information
  - Seems MNE-python can handle raw information so this may instead be better

**Objectives for Next Update:**

- Attempt to meet up with Jack (if he is available over the holidays) and obtain some data from the EEG cap
- Attempt implementation of incremental movement program for softhand

**Questions/Concerns:**

- Do we have to send project updates over the Easter break?

# Next Meeting

**Supervision Meetings:

**Progress on Actions:**

**Other Progress:**

**Objectives for Next Update:**

**Comments**

**Questions/Concerns:**
