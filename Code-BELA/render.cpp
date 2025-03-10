/*
TANGIBALLS LOOPER: Kevin Blackistone 2023-2025
Code for the Bela board.
Spherical audio looper for two spheres.



USES: \example Sensors/rotary-encoder/render.cpp

Audio filtering for Tangiballs loopers
*/

#include <Bela.h>
#include <math.h>
#include <vector>
#include <libraries/Scope/Scope.h>
#include <libraries/Encoder/Encoder.h>
#include <libraries/Biquad/QuadBiquad.h>
#include <libraries/AudioFile/AudioFile.h>
#include <libraries/math_neon/math_neon.h>


std::string gFilename[] = {"01.aif", "02.aif", "03.aif", "04.aif", "05.aif", "06.aif", "07.aif", "08.aif", "09.aif", "10.aif"};


// **************BIQUAD SECTION**********
// biquad filters
Biquad hpFilterA;
Biquad hpFilterB;
Biquad lpFilterA;
Biquad lpFilterB;
float lpFreq = 8000.0; 
float hpFreq = 500.0; 
float lpQ = 2.0;
float hpQ = 1.0;


// ***************ENCODER SECTIOJN***************
Encoder gEncoderXa;
Encoder gEncoderYa;
Encoder gEncoderXb;
Encoder gEncoderYb;
Scope gScope;

// Bela digital input channels connected to the encoder and button
unsigned int kEncX1a = 0;
unsigned int kEncX2a = 1;
unsigned int kEncY1a = 2;
unsigned int kEncY2a = 3;

unsigned int kEncX1b = 6;
unsigned int kEncX2b = 7;
unsigned int kEncY1b = 8;
unsigned int kEncY2b = 9;

// Record deactivate
unsigned int recButt = 4;
unsigned int switchTrackButt = 5;

// adjust the values below based on your encoder and wiring
unsigned int kDebouncingSamples = 5;		// originally 15, too high leads to aliasing errors, where it jumps between cw/ccw
Encoder::Polarity polarity = Encoder::ANY;	// could be ANY, ACTIVE_LOW, ACTIVE_HIGH


// ************ BUFFERY ***********************


// *********** GENERAL GLOBALS ****************
// Change in encoders per polling time of sumFrames (line 75-ish)
int xDifa = 0;
int yDifa = 0;
int xDifb = 0;
int yDifb = 0;

bool lastA = 0;
bool lastB = 0;

int sumFrames = 1024; // check every 8 blocks at 256 size ; //check 30/s  2940; // check value 15/sec
long long int lastTime = 0;

int lastXa = 0;
int lastYa = 0; 
signed int nowXa = 0;
signed int nowYa = 0;

int lastXb = 0;
int lastYb = 0; 
signed int nowXb = 0;
signed int nowYb = 0;



// ************ BUFFERY ***********************
#define PLAYBUFFERSIZE 44100 // four seconds to test - ~0.7Mb 
#define YBUFFERNUMBER 90
#define	FILEBUFFERS 10
int degreesPerBuffer = 4;

std::vector<float> inBuf;
float playBuffer[FILEBUFFERS][YBUFFERNUMBER][PLAYBUFFERSIZE];
int activeBuffera = 0;
int activeBufferb = 0;
int playBackCounter = 0;

float yFadea = 0.0;
float yMixLastBlocka = 0.0;
float yFadeb = 0.0;
float yMixLastBlockb = 0.0;


bool reversea = false;
bool mutea = true;
bool negativeFlipa = false;
bool reverseb = false;
bool muteb = true;
bool negativeFlipb = false;

bool switchTrackDown = false;
bool switchTrack = false;
bool switchPriorState = false;

float mix = 0.75;

// instantiate the scope
Scope scope;

int gAudioFramesPerAnalogFrame = 0;

float interpolateVal(int buf, float shiftedPtr, float inMix, float yFadeInterpolated, int yCh1, int yCh2, float audioIn){
			
			float sig = 0.0;	
		    	int indexBelow = floorf( shiftedPtr ); 

			// INTERPOLATE BETWEEN SAMPLES
			int indexAbove = indexBelow + 1;
			if (indexAbove >= PLAYBUFFERSIZE)
				indexAbove = 0;
			float fractionAbove = shiftedPtr - floorf(shiftedPtr);
			float fractionBelow = 1.0 - fractionAbove;
			
			// tempfrac = fractionAbove;	// for SCOPE
			sig = (playBuffer[buf][yCh1][indexBelow] * fractionBelow +
					playBuffer[buf][yCh2][indexAbove] * fractionAbove) * (yFadeInterpolated);
					
			sig += (playBuffer[buf][yCh1][indexBelow] * fractionBelow +
					playBuffer[buf][yCh2][indexAbove] * fractionAbove) * (1.0 - yFadeInterpolated);

			sig = (sig * (1.0-inMix)) + (audioIn * inMix);
			return sig;
}

bool setup(BelaContext *context, void *userData)
{

	if(context->analogFrames)
		gAudioFramesPerAnalogFrame = context->audioFrames / context->analogFrames;
		
	rt_printf("Loading Sample\n");
	int frameStart = 0;
	int useChannel = 0;
	rt_printf("Loading 2...\n");
	for (int i = 0; i < YBUFFERNUMBER; i++){
		std::vector<std::vector<float>> sampleData;
		for (int a = 0; a < FILEBUFFERS; a++){
			sampleData = AudioFileUtilities::load(gFilename[a], PLAYBUFFERSIZE, frameStart);
			for (int j = 0; j < PLAYBUFFERSIZE; j++){
				playBuffer[a][i][j] = sampleData[useChannel%sampleData.size()][j];
			}
		}
		frameStart += PLAYBUFFERSIZE;
	}
	
	rt_printf("...Loaded\n");
		
	// Set the digital pins to inputs
	pinMode(context, 0, kEncX1a, INPUT);
	pinMode(context, 0, kEncX2a, INPUT);
	pinMode(context, 0, kEncY1a, INPUT);
	pinMode(context, 0, kEncY1b, INPUT);
	
	pinMode(context, 0, kEncX1b, INPUT);
	pinMode(context, 0, kEncX2b, INPUT);
	pinMode(context, 0, kEncY1b, INPUT);
	pinMode(context, 0, kEncY2b, INPUT);

	pinMode(context, 0, recButt, INPUT);
	pinMode(context, 0, switchTrackButt, INPUT);
	
	// Setup the Biquads
	Biquad::Settings settingsHp{
			.fs = context->audioSampleRate,
			.type = Biquad::highpass,
			.cutoff = hpFreq,
			.q = hpQ,
			.peakGainDb = 0,
			};
	hpFilterA.setup(settingsHp);
	hpFilterB.setup(settingsHp);
	
	// Setup the Biquads
	Biquad::Settings settingsLp{
			.fs = context->audioSampleRate,
			.type = Biquad::lowpass,
			.cutoff = lpFreq,
			.q = lpQ,
			.peakGainDb = 0,
			};
	lpFilterA.setup(settingsLp);
	lpFilterB.setup(settingsLp);
	
	scope.setup(4, context->audioSampleRate);
	
	rt_printf("SETUP-COMPLETE\n");
	
	return true;
}

unsigned int holdoff;

float tempfrac;

void render(BelaContext *context, void *userData)
{
	for(unsigned int n=0; n<context->digitalFrames; n++){

        float out_l = 0.0;
        float out_r = 0.0;

		// READ ROTARY ENCODER PINS	
		bool x1a = digitalRead(context, n, kEncX1a);
		bool x2a = digitalRead(context, n, kEncX2a);
		bool y1a = digitalRead(context, n, kEncY1a);
		bool y2a = digitalRead(context, n, kEncY2a);
		
		bool x1b = digitalRead(context, n, kEncX1b);
		bool x2b = digitalRead(context, n, kEncX2b);
		bool y1b = digitalRead(context, n, kEncY1b);
		bool y2b = digitalRead(context, n, kEncY2b);
		
		// READ CONTROLS
		float inMix = analogRead(context, 0, 2);
		bool record = digitalRead(context, n, recButt);
		bool switchTrack = digitalRead(context, n, switchTrackButt);
		
		// SWITCH BETWEEN FILE BUFFER TRACKS
		// if (switchTrack) rt_printf("PRESS");
		if (switchTrack && !switchPriorState) {
			activeBufferb++;
			if (activeBufferb >= FILEBUFFERS) activeBufferb = 0;
			// rt_printf("Switching to RIGHT %d\n", activeBufferb);
			if (record) {
				activeBuffera++;
				if (activeBuffera >= FILEBUFFERS) activeBuffera = 0;
				// rt_printf("Switching to LEFT %d\n", activeBuffera);
			}
		}
		switchPriorState = switchTrack;
		
		
		// GET CURRENT ROTATIONS
		Encoder::Rotation retXa = gEncoderXa.process(x1a,x2a);
		Encoder::Rotation retYa = gEncoderYa.process(y1a,y2a);
		if(Encoder::NONE != retXa) nowXa = gEncoderXa.get();
		if(Encoder::NONE != retYa) nowYa = gEncoderYa.get();
		
		Encoder::Rotation retXb = gEncoderXb.process(x1b,x2b);
		Encoder::Rotation retYb = gEncoderYb.process(y1b,y2b);
		if(Encoder::NONE != retXb) nowXb = gEncoderXb.get();
		if(Encoder::NONE != retYb) nowYb = gEncoderYb.get();
		
		
		// AUDIO INPUT ... ADD TO RECORD BUFFER IF RECORDING
		float audioIn = audioRead(context, n, 0); // Left channel mono
		if (record) inBuf.push_back(audioIn); 
	
	
		// CHOOSE Y-MOTION CHANNEL NUMBER AND COMPUTE INTERPOLATION
		float yBuffera;
		if (nowYa >= 0) yBuffera = float(nowYa % 360);
		else yBuffera = float( (abs(nowYa) + 180) % 360);
		yBuffera = yBuffera / degreesPerBuffer;
		
		int yCha = int(floor(yBuffera));
		if (yCha >= YBUFFERNUMBER) yCha = 0;
		int fadeCha = (yCha + 1) % YBUFFERNUMBER;
		
		if (n == 0) {
			yMixLastBlocka = yFadea;
			yFadea = yBuffera - float(yCha);
		}
		
		float yBufferb;
		if (nowYb >= 0) yBufferb = float(nowYb % 360);
		else yBufferb = float( (abs(nowYb) + 180) % 360);
		yBufferb = yBufferb / degreesPerBuffer;
		
		int yChb = int(floor(yBufferb));
		if (yChb >= YBUFFERNUMBER) yChb = 0;
		int fadeChb = (yChb + 1) % YBUFFERNUMBER;
		
		if (n == 0) {
			yMixLastBlockb = yFadeb;
			yFadeb = yBufferb - float(yChb);
		}
		
		// INTERPOLATE BLOCK Y CROSSFADES, taking prior block fade % into consideration
		float yFadeInterpolateda = map((float)n/float(context->digitalFrames), 0.0, 1.0, yMixLastBlocka, yFadea);
		float yFadeInterpolatedb = map((float)n/float(context->digitalFrames), 0.0, 1.0, yMixLastBlockb, yFadeb);
		
		
		// SUMS TOTAL CHANGE AT LOWER FREQUENCY
		// RESOLUTION IS NOT GREAT ENOUGH FOR PER SAMPLE OR PER BLOCK ACCURACY
		// 1024 frames (45 windows / s) seems a good balance
		// Maybe sliding window for better response?
		if ( context->audioFramesElapsed - lastTime > sumFrames){
			
			lastTime = context->audioFramesElapsed;
			
			xDifa = nowXa - lastXa;
			yDifa = nowYa - lastYa;
			reversea = (xDifa < 0);
			mutea = (xDifa == 0);
			
			xDifb = nowXb - lastXb;
			yDifb = nowYb - lastYb;
			reverseb = (xDifb < 0);
			muteb = (xDifb == 0);
			
			
			// RECORD BLOCK WHEN ACTIVE AND MOVING (x only)
			if (record && (xDifa != 0)){
				
				// 122.5 = 44100 / 360 (ticks per rotation), to approximate audio frames per tick at 1sec / rotation
				int writeFrames = abs(xDifa * 128);	
				int writePtrStart = (nowXa % 360) * 128;
				while (writePtrStart < 0) writePtrStart = PLAYBUFFERSIZE + writePtrStart;
				
				for (int framePtr = 0; framePtr < writeFrames; framePtr++){

					int writePos = 0;
					float normalizedPtr = (float)framePtr / ((float)writeFrames);
					int readPos = int(normalizedPtr * inBuf.size());
					
					if (!reversea){
						writePos = (writePtrStart + framePtr) % PLAYBUFFERSIZE;
					}
					else {
						writePos = ((writePtrStart + writeFrames) - framePtr) % PLAYBUFFERSIZE;
					}
					// Crossfading between "tracks"
					float priorValue1 = playBuffer[activeBuffera][yCha][writePos];
					float priorValue2 = playBuffer[activeBuffera][fadeCha][writePos];
					
					playBuffer[activeBuffera][yCha][writePos] = 
						(inBuf.at(readPos) * (1.0f - yFadea)) 
						+ (priorValue1 * yFadea);
						
					playBuffer[activeBuffera][fadeCha][writePos] = 
						(inBuf.at(readPos) * yFadea)
						+ (priorValue2 * (1.0f - yFadea));
					
				}
				
				//rt_printf("X: %d\tΔX: %d\tY: %d\tΔY: %d\t Ch: %d  Buff: %2f In:%2f\n", 
				//		nowXa,	xDifa,	nowYa,	 yDifa, yCha, yBuffera, inMix);
					
				inBuf.clear();
				
			} else	{
				inBuf.clear();
			}
			
			lastXa = nowXa;
			lastYa = nowYa;
			
			lastXb = nowXb;
			lastYb = nowYb;
			
		}
		
		float blend = float(context->audioFramesElapsed - lastTime + n) / (float)sumFrames;

		float shiftedPtra = map(blend, 0.0, 1.0, (float)lastXa * 128.0, float(lastXa + xDifa - 1.0) * 128.0);
		shiftedPtra += 22050.0;	// OFFSET PLAY 'HEAD' 180° FROM RECORD 'HEAD'
		
		while (shiftedPtra >= PLAYBUFFERSIZE) 
			shiftedPtra -= PLAYBUFFERSIZE;
		while (shiftedPtra < 0)
			shiftedPtra += PLAYBUFFERSIZE;
			
		float shiftedPtrb = map(blend, 0.0, 1.0, (float)lastXb * 128.0, float(lastXb + xDifb - 1.0) * 128.0);
		shiftedPtrb += 22050.0;	// OFFSET PLAY 'HEAD' 180° FROM RECORD 'HEAD'
		
		while (shiftedPtrb >= PLAYBUFFERSIZE) 
			shiftedPtrb -= PLAYBUFFERSIZE;
		while (shiftedPtrb < 0)
			shiftedPtrb += PLAYBUFFERSIZE;
			
		
		if (!mutea) out_l = interpolateVal(activeBuffera, shiftedPtra, inMix, yFadeInterpolateda, yCha, fadeCha, audioIn);
		else out_l = audioIn * inMix;
		
		if (!muteb) out_r = interpolateVal(activeBufferb, shiftedPtrb, inMix, yFadeInterpolatedb, yChb, fadeChb, audioIn);
		else out_r = audioIn * inMix;
		
		out_l = hpFilterA.process(out_l);
		out_r = hpFilterB.process(out_r);
		out_l = lpFilterA.process(out_l);
		out_r = lpFilterB.process(out_r);
		
		audioWrite(context, n, 0, out_l);
		audioWrite(context, n, 1, out_r);
		
		// scope.log(out_r, out_l, yfi, yFadea);
		
	}
	
}

void cleanup(BelaContext *context, void *userData)
{
}

