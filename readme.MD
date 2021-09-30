# ST25DV tester

Some code to test ST25DV NFC chips using an arduino. Most interested in getting the fast transfer mailbox stuff to work.

## Quick start

1. Plug modified ST25DV eval board into the Arduino. It should connect to pins A5-A2 with the coil pointing away from the Arduino.
2. Load the skecth in repo onto the Arduino.
3. Open an Arduino serial monitor window with 1000000 baud. 
4. You should see a "Waiting for GPO..." message.
5. Run the sending app on the phone.
6. Hold the phone to the ST25DV coil.
7. You should see a file transfer progress print in the serial monitor. 
8. If the transfer suceeds, then you should see "download complete" and the expected and actual CRCs should match.
9. Process will restart after a success. In case of a failed download, press the RESET button on the Arduino to try again. 

## Connections

### Communications 

![image](https://user-images.githubusercontent.com/5520281/135495113-111eaf12-31ef-4298-8e93-bd8785e0ea90.png)

Note that SDA and GPO are connected together. We will set the GPO to pull down the pin when a new message arrives in the mailbox (RF PUT). This will wake us with an interrupt on the first one, then we will use it to know when to grab the next mailbox after each block is downloaded.  

Both SDA has 5K pull-up to Vcc. This is used during data transfer, but disabled when Vcc is floating - in which case we will use the internal pin pull up. Note we need this extra pull-up becuase the internal pullup is not strong enough for high speed data. 

It is easy to build this circuit on an ST25DV eval board...
![image](https://user-images.githubusercontent.com/5520281/135500422-8b78521a-6500-472a-b06e-2ef6f822a6b2.png)

The pins are laid out to make it possible to directly plug an ST25DV eval board directly into the Arduino header pins A5-A2. The board's coil should be pointing away from the Arduino. 


### Power

You can not enable the mailbox or use i2c features unless the chips sees good VCC, so we supply it via an IO pin in response to seeing activity from the GPO pin.


### Wake up

#### From MCU Side

1. RF field is presented to the ST25.
2. The ST25 pulls GPO pin low based on FIELD_CHANGE_EN. 
3. This wakes/interrupts the MCU.
4. MCU puts power on VCC.
5. MCU uses i2c to enable  dynamic GPO to signal on received mailbox message. 
6. MCU enables mailbox.
7. MCU puts welcome message into mailbox (should contain a cookie + high score info).
7. MCU waits for GPO signal that mailbox is ready.
8. MCU reads block from mailbox.
9. Back to #7 until all game blocks are received. 
10. MCU powers down Vcc.

#### From Phone Side

1. Phone is presented to ST25
2. Phone polls mailbox register looking for mailbox message ready. TODO: How fast should we poll?
3. Phone check for proper welcome message.
4. Phone puts block into mailbox. Note first block is special and has l;ength and CRC. 
5. Phone polls to see when message is read.
6. Back to #3 until all blocks are written.

I do not think we need or want a final confirm block from the MCU back to the phone? 

## Initialization

To save time on each connect we can get everything ready by setting stat registers....

MB_MODE = 0x01. Allows mailboxes to be enabled.

TODO: We probably want to put an NREF record that has a URI that points to the app in the app store? Can we support both iPhone and Android? Can we have it automatically open the app?  

TODO: i2c Password? Do we want to change this? Probably.  

TODO: We probably want to block write access to everything except the dyn mailbox register from the RF side to keep people from overwriting stuff?

TODO: Do we want the MCU to do initialization on every power up? Or do we want to program the ST25 via RF one during manufacture? If we do with MCU, one less factory step, but does uses more flash and means password is inside the MCU flash. 

## On connect

Assuming all of the above are set, we need to...

MB_CTRL_Dyn = 0x01. Turn on mailbox function. (Needs to be done at every power up). Faster to have the RF side do this so it can start sending for block as without waiting for anything from the i2c side? First handshake is seeing the mailbox get consumed?

## To transfer

RF writes blocks to the mailbox, i2c reads them and writes them to flash.

RF side knows when to write next block by polling on the RF_PUT_MSG bit to see when the i2c has read the last one out. TODO: How often to poll to not block the i2c side? 

TODO: Who starts? Do we try to make our own protocol, or try to reverse engineer the ST one? Copy block out of mailbox fast of i2c to buffer so next block can start sending while flash is burning.

TODO: What is the difference between the "fast" and normal message commands? Do the fast ones work on all phones? How do we check? 

## RF NFC  commands

### Read GPIO register

This is good to test if you have connectivity since it will return a prdictable non-zero number.

00 0A 02 00 

Should return 0x88 after power up. T

![image](https://user-images.githubusercontent.com/5520281/129462373-960936b2-941a-4025-8474-97f0320e382e.png)


00=Request flags (none)

### Read Dyn register

02 AD 02 0D

Returns 00 00 on reset, or 00 01 if mailbox enabled. Bit 1 to see if message ready from i2c. Bit 2 if a message from RF is pending (check this to see if all clear to send next block).

00=Request return status
01=The actual value of the read register

02=REQ_FLAG with bit #2 "high data rate" set
AD=Command code for read dynamic reg
02=Manufaturer ID for the customer command
0D=THe address of the MB_CTRL_Dyn register

### Write Dyn register

02 AE 02 0D 01

Enable mailbox. Returns 0x00 on success. I think probably the i2c side will do this so the RF side doesn't have to. 

### Read mailbox control reg

02 AD 02 0D

![image](https://user-images.githubusercontent.com/5520281/129467551-2290625b-7178-4ec0-a16b-3cd6bbd74ce0.png)

First we check the HOST_PUT_MSG bit to see if the high score block from the blink is in the mailbox yet. Poll until it is - but not too fast since access to the mailbox is exclusive. 

Then we read the mailbox with...

### Read mailbox block
First we read the len, then the data itself

#### Read message len 

02 AB 02

The response is the len.

### Read message data

02 AC 02 00 xx

...where xx is the len from above. The 00 is the starting byte index. Note that the actual data len is xx+1 so if XX is 0 then you will get 1 byte and is xx is ff then you get 256 bytes. 

### Write message

02 AA 02 xx dd dd dd ....

where xx is the (number of dd bytes) -1 

What is the difference between Write and Fast Write?

## iPhone Questions

Seems like the iPhone does not keep the RF field on between transactions, but we need the Vcc to stay up the whole time. TODO: Is there a way to keep that field up in the iOS API? Or do we need to add a capacitor there? Or even maybe need to waste a pin to power Vcc after connect so iPhone can do mailboxes? 

## Protocol

Not sure which side should start the conversation - maybe the phone sends a magic block to blink? If so, protocol something like...

1. Phone: Hi blink, this is the blink app
2. Blink: OK, here is a [app state block](#app-state-block) for you
3. Phone: I have a new game for you, it is X blocks long and the CRC is Y.
4. Blink: OK, give me block 0
5. Phone: Here is block 0
6. Blink OK, give me block 1
7. Phone: Here is block 1
...repeates until all blocks downloaded. 

If blink either (1) times out waiting for a block, or (2) gets a checksum mismatch then it goes into the bad download red error blink mode, just like an IR failed download. 

If phone can ditect a loss of NFC link, then it aborts with error screen. Otherwise it can use a very short (<1 sec) timeout waiting for next block request.

### App State Block

At the beginging of each NFC link, the blink will send a block to the phone with state data saved by different games. There will be a new API call on the NFC blink like `gameSaveData( const char *buffer , byte len )` that will update the info in this block. The blink will remember as many of the most recent saves as it can fit into a block. 

#### Block format

| Bytes  | Content |
| - | - |
| 1 | Record count. |
| n | Records |

#####  Record format

Data is arbitrary bytes. Each game can pick how long, and what to put in there. An example might be 4 byte high score + sequence of moves to get there. The ser

| Bytes | Content |
| - | - | 
| 2  | Game UID | 
| 1 | Data len |
| n | Data |

