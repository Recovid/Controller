# Controller
Sensors, electro-valves and controller

## Recovid-synoptique

![recovid-synoptique](image/high level synoptique.jpg)

## lexique
|    Nom|definition|range | resolution|
|----------------|-------------------------------|-----------------------------|-----------------------------|
| Paw | pression voie aerienne (en cmH20) |-2 ; 100| 0.1|
| QPatientSLM | debit patient a 21°C et 1013hPa|-200 ; 200|0.1|
| QPatientBTPS | debit Patient a 37°C, Pression ambiante et 100% Humidite (breath Temperature and pressure saturated)|-200 ; 200|0.1|
| RawO2 | tension qui sort de la cellule O2 |||
| PAtmo Pression atmospherique (exprime en hpa)|0-1200|1|
| Ti | Temps d'inspiration(seconde)  = VT(en L)/Debit de pointe(L/Min)|0;2|0.1|
| VTi | volume tidal inspiratoire (ml)|0-1500|1|
| VTe | Volume tidal expiratoire (ml)|0-1500|1|

## principe de fonctionement

### sensing
en continu (1kHz) on lit  Paw et QPatientSLM

en continu (1Hz) on lit RawO2 et PAtmo

* O2Concentration = RawO2*Gain
* QPatientBTPS = QPatientSLM * 1013/(Patmo-62,66) * (310°K/294°K)
* VTi = integral de QPatientBTPS durant toute l'inspiration et a l'expiration jusqu'a ce que le debit repasse en negatif
* VTe = integrale de QPatientBTPS durant le reste du temps (Q negatif et positif)
* VMi = somme des 8 derniers VTi * frequence respiratoire/8
* VMe = somme des 8 derniers VTe * frequence respiratoire/8
* PEP = moyenne de Paw durant les 100 derniere ms de l'expi
* PPlat = moyenne de Paw durant les 50 derniere ms du plateau ou de la pause inspiratoire
* PPeak = max de Paw durant inspiration + 100ms
* FiO2 = moyenne de O2Concentration sur le cycle

### therapie
Inspiration :
* on ferme l'electovalve I/E (Valve expi connecte a la sortie du ballon)
* on actionne le ballon pendant le temps Ti a la vitesse determiné (pas de controle durant le cycle)
* apres le Ti on commence a faire revenir le ballon mais on garde l'electrovanne fermé durant le temps de plateau ou le temps de la pause inpiratoire (tant que le medecin a le bouton appuyé) puis passage a l'expiration.

Expiration
* On ouvre l'electrovanne (plutot simple)
* si le medecin appuie sur pause expiratoire on ferme la l'electrovanne tant qu'il est appuye dessus

### regulation
cycle a cycle pour la PEP et le VT
* on ajuste la commande de PEP une fois qu'on a la mesure du cycle (a la fin de l'expi pour avoir le temps d'atteindre la pression de commande)
* pour le VT on va ajuster le nombre de tour de consigne (attention a ne pas compenser tout d'un coup)

Pour le moment je pense qu'il faut avoir une vitesse constante. je ne pense pas qu'il soit necessaire de viser une forme de debit precise.

###Calibration O2
c'est une procedure a part ou l'on met la cellule O2 à l'air (pendant 2minute au minimum) et on prend la valeur RawO2 et Patmo a la fin des 2 minutes. a faire avant le debut de la ventilation probablement en assurant un passage d'air devant la cellule pour flusher toute trace d'O2.

## Sensors
### Airflow Sensor
#### Embedded high end solution
[Sensirion SFM3300-AW](https://fr.farnell.com/sensirion/sfm3300-aw/flow-sensor-washable-95ac5341/dp/3103639?st=sfm3300)

[Application Note I2C](https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/5_Mass_Flow_Meters/Application_Notes/Sensirion_Mass_Flo_Meters_SFM3xxx_I2C_Functional_Description.pdf)
#### DIY solution
[Pneumotach](https://www.hamilton-medical.com/dam/jcr:b8ef47c8-c2fa-47d5-8378-7fb9198ff7fc/Flow-sensor-tech-specs-EN-689568.00.pdf) setup with a differential pressure sensor. According to the datasheet, de pressure difference is around 5mbar @ 180L/min.

Differential pressure sensors ordered:

[HSCDRRD006MDSA3](https://eu.mouser.com/ProductDetail/Honeywell/HSCDRRD006MDSA3?qs=Yk42LiOZU8RCThK5r%252BnSIA==) SPI, +-6mbar

[SP610-500pa](https://fr.farnell.com/sensirion/sdp610/capteur-pressure-i2c-500pa/dp/1696081) I2C, +-5mbar

### Paw sensor
Honeywell [HSC series](https://sensing.honeywell.com/honeywell-sensing-trustability-hsc-series-high-accuracy-board-mount-pressure-sensors-50099148-a-en.pdf) (all selected are i2c):

1psi differential: HSCDDRD001PD2A3, HSCDRRN001PD2A3

1psi gage: HSCDLND001PG2A3, HSCDRRD001PG2A5

Honeywell [APB series](https://sensing.honeywell.com/honeywell-sensing-basic-board-mount-pressure-abp-series-datasheet-32305128.pdf):

[ABPLANT001PG2A5](https://fr.farnell.com/honeywell/abplant001pg2a5/capteur-pression-i2c-1psi-axial/dp/2773681)

Amphenol:

1psi differential : [NPA-700B-001D](https://fr.farnell.com/amphenol-advanced-sensors/npa-700b-001d/capteur-pression-numerique-1psi/dp/2845670?sf=714&pf=211704018%2C210151241%2C213626898%2C210393238%2C210077320%2C210275905%2C210120154%2C210173108%2C213170980%2C210185510%2C210123539%2C210201779%2C210141386%2C210162491&krypto=tXn4MW9f88KqMhIg%2B0OkCbKwAiudqEmBcvXgKv8d3qew%2BiAz%2BJVJo42AaofJc1K9OY56N829Im1iHXDzwJi9oWvJ5naMyMpCGMUAYn42T1dK%2FBWRNZsY1eK149kw4ydT7co85n1jXgqJl6gMef0A6ftCqyAp1yo2rgjzznggHJiuxJzXsq2TdlIHwr%2B9blfPmwwJeg59CGqnEMbPn6CP6f41PcuxhSX9wXl9FHrpTwSw1mBnIxsTBAPXiTgCwZzRzQ5h2JuzzwoTmlmyIco3btIFm3d8GfP8S7ShM6BXanuwlxQzl%2Bw3CCstCr5bKyKA8tvmTRKhrdxWWgHm3SFu6lh%2FQUGmdo481sgvtreK2kNm1EXP5dvBTyPV9%2Fxf64s%2FKw7Uxsv8vsl5QnxORxGeTgHHxZyqMaysMqKM%2F9EP364DtFrPgrpH1jplq78y5x2AURYP2I6avr2PKIiBzckP%2BQvyi1YEUH0C5rZoKe7i4Zyw6shg6SQCP%2F37EgmmfFqe1jknBVsdvZg7DbrUm4q99d7TK0Dl%2F7aCQl%2FYfjkfCl21mVVY0xAi5uDRLkL0%2FZYXBdZYLX6rjHRj%2B2npSQ0a%2Bc5ujp%2B7ZdNLgJR%2BTZomjlaISs3o%2BRJN4okOpbqpJ4IVhU%2FUCtO3Sg%2F1q6TMFH33JL%2BEwVPD%2FO3cOSPKGPyNkm07gZky48XFFqtvLM96J0fpGu8VEuJXkpalzeV3npV7CYdmCXc6kzSuETu1Ao%2BqyaDmUMNjOwniZL4%2BEyEKmKJo8THhWXkPRhfzLz4oJCOAoIME0sq7ytBUyh%2BPjvNjJVuG1xd%2FKufoFseFgrvwDzSe9H%2Bsh87G81gEfZcaE7UlABAaCDrv2jMVP9AmvPiIqGErWtDys%2F0ZDdAVLVvjIhM%2F7jloMldfPbbETZ%2B9poPEePJf9eURPT0uRXcJW28fwKN5ZbGSNw1nMfFong5T98FaKJMDurnzfgsbCOUUmUyiN9W3FwJVkKrRN92lUmbFRScqepaWbi0FwKIXZ%2FI9CEQNdV5fpllcnzBws03k%2FNOU8a9XFG5GIsST7Rc3EajgG%2BRF1WftDZ1E9ObcAI%2BxPEveq5abrCEX4J0S%2Bi5%2FTPAZcx2GCydjqhjo69B1NVmL5IjsABzbouQ3gNQSyspqLmbdKIu1tQM4RwDP4CrMXVS6KjZ7TBdtP2o%2FNSHFpSFJN%2BneslzLknmCoVmbEEjFxNHt3Yc6e6KUAvCa%2FS4YH9p%2FdoCdx%2BMxqcnmIDXmhyIbKj2wsq031N8iolJ93OWzx2PafFWJ%2BaVf7ZghkvyE8lD3i3dgFpMyOZDH9f0%2BZ17JeIxEzCFvzZp89iwBr0GRYKtjfnNodpSlJsinJ6gdAcbHNFTtxQ4pLrR7%2Fg%2Bq3iiCN6HHxFBBSaWl%2F1gshXhweHqBxUa3YjhpoOeXkzjsch3n%2B1gFUAGr3QobcHwcCWMN4SJQ1Dp6ewQN8Wl7MhWv5mq6XCj7iR8eZSVZuza9jbLWf2Y9i5WmHBankh59ZMHRD9WtGdMjBW7LYphgMfoxPF9A5c%2FFUVq6L1DOSJksse%2FfAfVlVceG%2ByVHxqLJ9RQEMhzKNFLtXeoYEAO6iiQrdjiGOHWRKIcvsosuWDhWIQqZ1%2FoHaX3caqg%2F0k%2BLQ8vMau2zo%2FDmJp54ckQzF8bymO48mFOCluxW8hXtasqWd%2BJK02D9RC0fvcxixkLHAbJE3NJouaqhculTRIEAwtmo%2Fcsoax7eZLpL3TBKSwswwwNW9fAQMMo5bbu6ArARh9b5gX4jqSak2eX5HCzwr1iJFQbOSelgqCYraX5AdR4RQ4e6wxlFT47CEAkUGlonBYB7pj%2BaJc3iec7k%2FMgcDFKH6GX%2F%2BfPr4KztA2fQlUfmxn7WOJAKByceJwbzJz0KTNaRC5RAznzVVjsRYbQmKPxlOQLl&ddkey=https%3Afr-FR%2FElement14_France%2Fw%2Fc%2Fcapteurs-transducteurs%2Fcapteurs%2Ftransducteurs-capteurs-de-pression%2Ftransducteurs-de-pression)

1psi gage : [NPA-700B-001G](https://fr.farnell.com/amphenol-advanced-sensors/npa-700b-001g/capteur-pression-numerique-1psi/dp/2845671)

### Patmo, Tatmo sensor
All are I2C/SPI
[MM5611](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5611-01BA03&DocType=Data+Sheet&DocLang=English)

Bosh [bmp280](https://www.bosch-sensortec.com/media/boschsensortec/downloads/documents/product_related_documents/product_flyers/bst-bmp280-fl000.pdf)

Bosh [bme280](https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/)
