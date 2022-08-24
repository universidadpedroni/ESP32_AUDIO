close all;clear all; clc
fundamental_freq = 687.5;

FREQ_OF_NOTES = [16.35 17.32 18.35 19.45 20.60 21.83 23.12 24.50 25.96 27.50 29.14 30.87];
NOMBRES = ['C '; 'C#';'D '; 'D#';'E '; 'F '; 'F#';'G '; 'G#';'A '; 'A#';'B '];
CANTIDAD_DE_OCTAVAS = 8;
DETUNE = 0.02;

for octava = 0:CANTIDAD_DE_OCTAVAS
    disp('====')
    for nota=1:12
        fprintf('%s%i %.2f \t %.2f \t %.2f \n',...
            NOMBRES(nota,:),...
            octava,...
            2^octava * (1 - DETUNE) * FREQ_OF_NOTES(nota),...                
            2^octava * (1) * FREQ_OF_NOTES(nota),...
            2^octava * (1 + DETUNE) * FREQ_OF_NOTES(nota));

    end
    
end

    

