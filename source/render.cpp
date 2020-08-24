// Assignment 1: synth filter
//
// Jack Walters
//
// ECS732 Real-Time DSP
// School of Electronic Engineering and Computer Science
// Queen Mary University of London
// Spring 2019

// Include relevant header files
#include <Bela.h>
#include <libraries/Gui/Gui.h>
#include <libraries/GuiController/GuiController.h>
#include <cmath>

const int gSampleBufferLength = 512;		// The length of the buffer in frames
float gSampleBuffer[gSampleBufferLength];	// Buffer that holds the wavetable
float gReadPointer = 0;						// Position of the last frame we played

const int kSensorInputCuttOffFrequency = 0;
const int kSensorInputQ = 1;

// Initialise filter 1 state variables
float F1out0 = 0;
float F1gLastOutput = 0;
float F1gLastLastOutput = 0;
float F1gLastInput = 0;
float F1gLastLastInput = 0;

// Initialise filter 2 state variables
float F2out0 = 0;
float F2gLastOutput = 0;
float F2gLastLastOutput = 0;
float F2gLastInput = 0;
float F2gLastLastInput = 0;

// Initialise coefficient variables
float a0 = 0;
float b0 = 0;
float b1 = 0;
float b2 = 0;
float a1 = 0;
float a2 = 0;

// Browser-based GUI to adjust parameters
Gui gui;
GuiController controller;

// Calculate filter coefficients given specifications
void calculate_LPcoefficients(float sampleRate, float frequency, float q)
{
	// Declare T and Omega0 as variables local to calculate_LPcoefficients() function
	const float T = 1/sampleRate;
	const float Omega0 = 2*M_PI*frequency;
	
	// Update global coefficient values to be used in difference equation in render() function
	b0 = (pow(Omega0,2)*q*pow(T,2));
	b1 = (2 * pow(Omega0,2)*q*pow(T,2));
	b2 = (pow(Omega0,2)*q*pow(T,2));
	
	a0 = (4*q + 2*Omega0*T + pow(Omega0,2)*q*pow(T,2));
	a1 = (-8*q + 2*pow(Omega0,2)*pow(T,2)*q);
	a2 = (4*q - 2*Omega0*T + pow(Omega0,2)*q*pow(T,2));
}

void calculate_HPcoefficients(float sampleRate, float frequency, float q)
{
	// Declare T and Omega0 as variables local to calculate_HPcoefficients() function
	const float T = 1/sampleRate;
	const float Omega0 = 2*M_PI*frequency;
	
	// Update global coefficient values to be used in difference equation in render() function
	b0 = 4*q;
	b1 = -8*q;
	b2 = 4*q;
	
	a0 = (4*q + 2*Omega0*T + pow(Omega0,2)*q*pow(T,2));
	a1 = (-8*q + 2*pow(Omega0,2)*pow(T,2)*q);
	a2 = (4*q - 2*Omega0*T + pow(Omega0,2)*q*pow(T,2));
}

// Read an interpolated sample from the wavetable
float wavetable_read(float sampleRate, float frequency)
{
	// The pointer will take a fractional index. Look for the sample on
	// either side which are indices we can actually read into the buffer.
	// If we get to the end of the buffer, wrap around to 0.
	int indexBelow = floorf(gReadPointer);
	int indexAbove = indexBelow + 1;
	if(indexAbove >= gSampleBufferLength)
		indexAbove = 0;
	
	// For linear interpolation, we need to decide how much to weigh each
	// sample. The closer the fractional part of the index is to 0, the
	// more weight we give to the "below" sample. The closer the fractional
	// part is to 1, the more weight we give to the "above" sample.
	float fractionAbove = gReadPointer - indexBelow;
	float fractionBelow = 1.0 - fractionAbove;
	
	// Calculate the weighted average of the "below" and "above" samples
    float out = (fractionBelow * gSampleBuffer[indexBelow] + fractionAbove * gSampleBuffer[indexAbove]);

    // Increment read pointer and reset to 0 when end of table is reached
    gReadPointer += gSampleBufferLength * frequency / sampleRate;
    while(gReadPointer >= gSampleBufferLength)
        gReadPointer -= gSampleBufferLength;
        
    return out;
}

bool setup(BelaContext *context, void *userData)
{
	// Generate a sawtooth wavetable (a ramp from -1 to 1)
	for(unsigned int n = 0; n < gSampleBufferLength; n++) {
		gSampleBuffer[n] = -1.0 + 2.0 * (float)n / (float)(gSampleBufferLength - 1);
	}

	// Set up the GUI
	gui.setup(context->projectName);
	controller.setup(&gui, "Filter Controller");	
	
	// Arguments: name, default value, minimum, maximum, increment
	// Create sliders for oscillator and filter settings
	controller.addSlider("Oscillator Frequency", 220, 55, 440, 0);
	controller.addSlider("Oscillator Amplitude", 0.05, 0, 0.2, 0);
	controller.addSlider("Low-pass/High-pass", 0, 0, 1, 1);    // Create slider which toggles low-pass (0) or high-pass (1)
	
	// The two controller class methods below are used when the user controls filter cutoff and Q through the GUI
	// controller.addSlider("Filter Cutoff Frequency", 1000, 100, 4000, 0);
	// controller.addSlider("Filter Q", 2, 0.5, 10, 0);

	return true;
}

void render(BelaContext *context, void *userData)
{
	// Read the slider values at frame 0 of each block
	float oscFrequency = controller.getSliderValue(0);	
	float oscAmplitude = controller.getSliderValue(1);
	int LPorHP = controller.getSliderValue(2);
	
	// The two controller class methods below are used when the user controls filter cutoff and Q through the GUI
	// float filterFrequency = controller.getSliderValue(2);
	// float filterQ = controller.getSliderValue(3);
	
	// Read the analog potentiometer values at frame 0 of each block
	float reading0 = analogRead(context, 0, kSensorInputCuttOffFrequency);
	float reading1 = analogRead(context, 0, kSensorInputQ);
	float linearfrequency = map(reading0, 0, 1, 2, 3.602);	// log10(4000) is 3.602, therefore our potentiometer value will transition linearly from 0-3.602
	float filterFrequency = pow(10,linearfrequency);	// 10 is raised to the power of a user-defined value from 0-3.602, creating logarithmic frequency mapping
	float filterQ = map(reading1, 0, 1, 0.5, 10);	// filterQ variable is mapped from 0.5-10
	
	// Calculate either low-pass or high-pass coefficient values, depending on whether the LPorHP variable is 0 or 1 respectively
	if(LPorHP == 0) {
		calculate_LPcoefficients(context->audioSampleRate, filterFrequency, filterQ);
	} else {
		calculate_HPcoefficients(context->audioSampleRate, filterFrequency, filterQ);
	}
	
    for(unsigned int n = 0; n < context->audioFrames; n++) {
    	// Read the next interpolated sample from the wavetable
		float in = oscAmplitude * wavetable_read(context->audioSampleRate, oscFrequency);
		// float in = audioRead(context, n, 0);

		// Filter 1 difference equation computation
		float F1out = ((
        	(b0 * in) + (b1 * F1gLastInput) + (b2 * F1gLastLastInput))
        	- 
        	((a1 * F1gLastOutput) + (a2 * F1gLastLastOutput)))
        	/a0;
        	
        F1gLastLastInput = F1gLastInput;	// x[n-2] = x[n-1]
		F1gLastInput = in;    // x[n-1] = x[n]
		
		F1gLastLastOutput = F1gLastOutput;    // y[n-2] = y[n-1]
		F1gLastOutput = F1out;    // y[n-1] = y[n]
		
		// Filter 2 difference equation computation, using filter 1's output as its own input
		float out = ((
        	(b0 * F1out) + (b1 * F2gLastInput) + (b2 * F2gLastLastInput))
        	- 
        	((a1 * F2gLastOutput) + (a2 * F2gLastLastOutput)))
        	/a0;
        	
        F2gLastLastInput = F2gLastInput;	// x[n-2] = x[n-1]
		F2gLastInput = F1out;    // x[n-1] = x[n]
		
		F2gLastLastOutput = F2gLastOutput;    // y[n-2] = y[n-1]
		F2gLastOutput = out;    // y[n-1] = y[n]
            
        // Write the output to every audio channel
    	for(unsigned int channel = 0; channel < context->audioOutChannels; channel++) {
    		audioWrite(context, n, channel, out);
    	}
    }
}

void cleanup(BelaContext *context, void *userData)
{

}
