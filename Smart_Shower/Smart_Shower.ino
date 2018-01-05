
// BLUETOOTH + PROFIELEN
#include <SoftwareSerial.h>						// bibliotheken voor bluetooth
SoftwareSerial BTserial(10, 11);				// RX en TX toewijzen 

int importProfiel = 0;							// Welk profiel er ingeladen wordt
char data[255];									// Alle data die ontvangen of verstuurd wordt in een array

// PROFIELEN DATA
int profielen = 6;								// Het aantal profielen in het systeem
int tijdFase[6][5];								// Multidimensionele Array: De tijd per fase per profiel. [6] is het aantal profielen, [5] is de tijd per fase
int dTemperatuurMinGrens[6];
int dTemperatuurMaxGrens[6];
int temperatuurMinFase[6][5];					// Multidimensionele Array: [6] is het aantal profielen, [5] is de minimum temperatuur per fase
int temperatuurMaxFase[6][5];					// Multidimensionele Array: [6] is het aantal profielen, [5] is de maximum temperatuur per fase

// DATA PER DOUCHE
unsigned long beginTijdDouche = 0;				// Begintijdstip van de douche
unsigned long eindTijdDouche = 0;				// Eindtijdstip van de douche
unsigned long faseovergangTijdstippen[255];		// Het tijdstip van een faseovergang (vooral manueel voor override van de temperatuur)
int totaleWaterverbruik = 0;					// Het totale waterverbruik
int verloopWaterverbruik[255];					// Het verloop van het waterverbruik over heel de douche. 
int gemiddeldeTemperatuur = 0;					// De gemiddelde temperatuur van heel de douche.
int verloopTemperatuur[255];					// Het verloop van de temperatuur over heel de douche.

int nTemperatuurMinGrens[5];					// De nieuwe temperatuur minimum grens per fase, gebruikt om de data te verwerken en het doucheritme beter te kunnen bepalen. 
int nTemperatuurMaxGrens[5];					// De nieuwe temperatuur maximum grens per fase, gebruikt om de data te verwerken en het doucheritme beter te kunnen bepalen. 

// TIJDELIJKE DATA 
int temperatuurMinGrens;						// De minimumgrens van de temperatuur, tijdelijke data
int temperatuurMaxGrens;						// De maximumgrens van de temperatuur, tijdelijke data

// ALGEMEEN
static int fase = 0;							// In welke fase zitten we, gebruikt in de case
int gekozenProfiel = 0;							// Het gekozen profiel voor de huidige douchebeurt

int sTemperatuurMinFase[] = { 0, 5, 2, 5, 0 };	// De standaard waarden voor de minimumtemperatuur per fase
int sTemperatuurMaxFase[] = { 0, 5, 2, 5, 0 };	// De standaard waarden voor de maximumtemperatuur per fase

int manueelBijsturenTeller = 0;					// Het aantal keren dat er al manueel is bijgestuurd geweest, gebruikt bij dataverwerking
int manueelBijsturenTijd[6];					// Op welk tijdstip werd er bijgestuurd, gebruikt bij dataverwerking + bij faseverloop (?) 
int manueelBijsturenTemp;						// Bij welke temperatuur werd er bijgestuurd, gebruikt bij dataverwerking
int manueelBijsturenDelay = 0;					// Met hoeveel tijd de fase verlengt wordt.
int manueelBijgestuurd = 0;						// Is er al bijgestuurd geweest? Belemmeren dat er niet bij elke temperatuuraanpassing er wordt verandert van fase.

// ENCODER
#include <ClickEncoder.h>
#include <TimerOne.h>

ClickEncoder *encoder;
int16_t lastEncoderValue, encoderValue;

// LEDS
#include <Adafruit_NeoPixel.h>

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN            6

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      6

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int colorsR[] = { 255, 255,   0,   0,   0, 255 };
int colorsG[] = { 0, 255, 255, 255,   0,   0 };
int colorsB[] = { 0,   0,   0, 255, 255, 255 };

//   Water Temperature DS18B20  //
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into pin 6 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature tempH20Sensor(&oneWire);
int tempH20 = 0;

// POTENTIOMETERS
int potWater = 0;
int potTemp = 0;
int potWaterPin = 3;
int potTempPin = 2;			// Moet ofwel pin 2 of 3 zijn door interrupt.

void timerIsr()
{
	encoder->service();
}

void setup()
{
	// BLUETOOTH
	BTserial.begin(9600);               //synchronisatiesnelheid bluetooth op 9600 baud
	Serial.begin(9600);

	// LED + ENCODER
	pixels.begin(); //
	encoder = new ClickEncoder(A1, A0, A2);
	Timer1.initialize(1000);
	Timer1.attachInterrupt(timerIsr);
	lastEncoderValue = -1;

	//   Water Temperature DS18B20  //
	tempH20Sensor.begin();

	attachInterrupt(digitalPinToInterrupt(potTempPin), temperatuurManueelBijsturen, CHANGE);		// Interrupt het programma om direct de temperatuur aan te passen. 
}

void loop()
{
	//importData();
	switch (fase)
	{
	case 0: // Voor het douchen begint
		profielSelecteren();
		startDouche();
		break;
	case 1: // Ontspannen in de douche
		dataLezen();
		temperatuurBijsturen();
		eindFases();
	case 2: // Haar/Lichaam inzepen
		dataLezen();
		temperatuurBijsturen();
		eindFases();
	case 3: // Afspoelen haar/lichaam
		dataLezen();
		temperatuurBijsturen();
		eindFases();
	case 4: // Lichaam/Haar inzepen
		dataLezen();
		temperatuurBijsturen();
		eindFases();
	case 5: // Afspoelen
		dataLezen();
		temperatuurBijsturen();
		eindFases();
	}
}

// DATATRANSFER BLUETOOTH, UNFINISHED

/*
void importData()							// UNFINISHED!!!!
{
	if (BTserial.available())
	{
		for (int i = 0; !BTserial.available()  Zolang er data verzonden wordt ; i++)
		{
			data[i] = BTserial.read();          //lees de data binnen als ASCII (karakter)
		}

		/* IMPORT/UPDATE DATA:
		Pakket bestaat uit: Welk profiel, Kleur, Ideale temperatuur, grenzen temperatuur, sterkte waterstraal. "
	
		for (int i = 0; )
			importProfiel = data[0];
		// EVENTUEEL KLEUR
		dTemperatuurMaxGrens[importProfiel] = data[1];
		dTemperatuurMinGrens[importProfiel] = data[2];
		// EVENTUEEL STERKTE WATERSTRAAL
		for (int x = 0; x < profielen; x++)
		{
			for (int y = 0; y)
		}
		tijdFase[importProfiel][0] = data[3];
		tijdFase[1] = data[4];
		tijdFase[2] = data[5];
		tijdFase[3] = data[6];
		tijdFase[4] = data[7];
	}
}

void exportData()							// UNFINISHED!!!!
{
	if (BTserial.available()) // Smartphone/server stuurt een request naar de douche voor data
	{
		BTserial.print("a" /* TEMP, WATERVERBRUIK, ... );

	}
}

*/

// CASE 0

void profielSelecteren()
{
	encoderValue += encoder->getValue();
	if (encoderValue < 0)
	{
		encoderValue = 59;
	}
	else if (encoderValue > 59)
	{
		encoderValue = 0;
	}
	if (encoderValue != lastEncoderValue) {
		lastEncoderValue = encoderValue;
		profielLed();
	}
}

void profielLed()
{
	gekozenProfiel = round(encoderValue / 10);
	int l[6];
	for (int i = 0; i<NUMPIXELS; i++)
	{
		l[i] = gekozenProfiel - i;
		if (l[i] < 0)
		{
			l[i] = l[i] + 6;
		}
	}
	for (int i = 0; i<NUMPIXELS; i++)
	{
		pixels.setPixelColor(l[i], pixels.Color(colorsR[i], colorsG[i], colorsB[i]));
		pixels.show();
	}
	pixels.show();
}

void startDouche()
{
	potWater = analogRead(potWaterPin);
	if (potWater >= 1000)
	{
		for (int i = 0; i = 5; i++)		// Als er nog geen waarden zijn, gebruik de standaardwaarden
		{
			if (temperatuurMinFase[gekozenProfiel][i] == 0)
			{
				temperatuurMinFase[gekozenProfiel][i] = sTemperatuurMinFase[i];
			}
			if (temperatuurMaxFase[gekozenProfiel][i] == 0)
			{
				temperatuurMaxFase[gekozenProfiel][i] = sTemperatuurMaxFase[i];
			}
		}
		beginTijdDouche = millis();
		fase = 1;
	}
}

// ALGEMEEN

void dataLezen()
{
	// TEMPERATUUR 
	tempH20Sensor.requestTemperatures();
	tempH20 = tempH20Sensor.getTempCByIndex(0);

	//WATERVERBRUIK

}

void temperatuurBijsturen()
{
	if (tempH20 <= temperatuurMinGrens - temperatuurMinFase[gekozenProfiel][fase - 1])
	{
		// BIJSTUREN TEMPERATUUR WARMER
	}
	else if (tempH20 >= temperatuurMaxGrens - temperatuurMaxFase[gekozenProfiel][fase - 1])
	{
		// BIJSTUREN TEMPERATUUR KOUDER
	}
}// Multidimensionele Array: [6] is het aantal profielen, [5] is de minimum temperatuur per fase

void temperatuurManueelBijsturen()
{
	manueelBijsturenTijd[manueelBijsturenTeller] = millis();
	manueelBijsturenTemp = tempH20;
	int potTempNew = analogRead(potTempPin);
	potTemp = potTemp - potTempNew;					// OF potTempNew - potTemp, TESTEN!! + TERUG NAAR VORIGE FASE???
	temperatuurMinGrens = temperatuurMinGrens + (potTemp * 0.01);
	temperatuurMaxGrens = temperatuurMaxGrens + (potTemp * 0.01);
	if (((fase == 2) || (fase == 4)) && (manueelBijgestuurd == 0))		// Temperatuur wordt in deze verlaagt, dus ofwel te vroeg naar volgende fase gegaan ofwel sneller gedaan met inzepen.
	{
		if (manueelBijsturenTijd[manueelBijsturenTeller] - beginTijdDouche < tijdFase[gekozenProfiel][fase] / 3)
		{
			fase--;
			manueelBijsturenDelay = manueelBijsturenDelay + 30000;
			manueelBijgestuurd = 1;
		}
		else if (manueelBijsturenTijd[manueelBijsturenTeller] - beginTijdDouche > tijdFase[gekozenProfiel][fase] / 3)
		{
			fase++;
			manueelBijsturenDelay = manueelBijsturenDelay - (manueelBijsturenTijd[manueelBijsturenTeller] - beginTijdDouche);
			manueelBijgestuurd = 1;
		}
	}
	if (((fase == 1) || (fase == 3)) && (manueelBijgestuurd == 0))
	{
		nTemperatuurMaxGrens[fase] = temperatuurMaxGrens;
		nTemperatuurMinGrens[fase] = temperatuurMinGrens;
	}
	if ((fase == 5) && (manueelBijgestuurd == 0))
	{
		nTemperatuurMaxGrens[fase] = temperatuurMaxGrens;
		nTemperatuurMinGrens[fase] = temperatuurMinGrens;
	}
	manueelBijsturenTeller++;
}

void eindFases()
{
	if (beginTijdDouche + tijdFase[gekozenProfiel][fase] + manueelBijsturenDelay <= millis())
	{
		fase = fase++;
	}
}
