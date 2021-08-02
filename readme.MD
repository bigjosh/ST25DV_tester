# ST25DV tester

Some code to test ST25DV NFC chips using an arduino. Most interested in getting the fast transfer mailbox stuff to work.

## Connections

### Communications 

```
ST25 SCL -> Arduino A5
ST25 SDA -> Arduino A4
ST25 Ground -> Arduino Ground.
```
Both SCL and SDA have 1K pull-ups to Vcc.

### Power

```
ST25 EH -> ST25 VCC
```

You can not enable the mailbox features unless the chips sees VCC, so we grab the VCC from the energy harvesting pin, and then we enable energy harvesting.

### Wake up

~100K pull-down from SDA to ground.

This bleeds off any charge on SDA when there is no RF connection to the chip. This makes it so we can detect 
a new connection when ever SDA goes high (because it will be driven by EH).

This resistor is big enough that it should not matter when things are running normally.

## Initialization

To save time on each connect we can get everything ready by setting stat registers....

MB_MODE = 0x01. Allows mailboxes to be enabled.
EH_MODE = 0x00. Turns on energy harvesting each time we get an RF power up.

TODO: We probably want to put an NREF record that has a URI that points to the app in the app store? Can we support both iPhone and Android? Can we have it automatically open the app?  

TODO: i2c Password? Do we want to change this? 

TODO: We probably want to block write access to everything except the dyn mailbox register from the RF side to keep people from overwriting stuff? 

## On connect

Assuming all of the above are set, we need to...

MB_CTRL_Dyn = 0x01. Turn on mailbox function. (Needs to be done at every power up). Faster to have the RF side do this so it can start sending for block as without waiting for anything from the i2c side? First handshake is seeing the mailbox get consumed?

## To transfer

RF writes blocks to the mailbox, i2c reads them and writes them to flash.

RF side knows when to write next block by polling on the RF_PUT_MSG bit to see when the i2c has read the last one out. TODO: How often to poll to not block the i2c side? 

TODO: Who starts? Do we try to make our own protocol, or try to reverse engineer the ST one? Copy block out of mailbox fast of i2c to buffer so next block can start sending while flash is burning.

TODO: What is the difference between the "fast" and normal message commands? Do the fast ones work on all phones? How do we check? 

## iPhone Questions

Seems like the iPhone does not keep the RF field on between transactions, but we need the Vcc to stay up the whole time. TODO: Is there a way to keep that field up in the iOS API? Or do we need to add a capacitor there? Or even maybe need to waste a pin to power Vcc after connect so iPhone can do mailboxes? 
