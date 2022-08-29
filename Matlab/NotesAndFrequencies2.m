close all;clear all; clc
fundamental_freq = 687.5;

FREQ_OF_NOTES = [16.35 17.32 18.35 19.45 20.60 21.83 23.12 24.50 25.96 27.50 29.14 30.87];
NOMBRES = ['C '; 'C#';'D '; 'D#';'E '; 'F '; 'F#';'G '; 'G#';'A '; 'A#';'B '];
CANTIDAD_DE_OCTAVAS = 8;
DETUNE = 0.025;

fprintf('Nota \t Midi/Freq \t Midi/Freq \t Midi/Freq \t');
for octava = 0:CANTIDAD_DE_OCTAVAS
    
    for nota=1:12
        fprintf('%s%i \t %.0f %.2f \t %.0f %.2f \t %.0f %.2f \n',...
            NOMBRES(nota,:),...
            octava,...
            69 + 12 * log2(2^octava * (1 - DETUNE) * FREQ_OF_NOTES(nota)/440),...
            2^octava * (1 - DETUNE) * FREQ_OF_NOTES(nota),...                
            69 + 12 * log2(2^octava * FREQ_OF_NOTES(nota)/440),...
            2^octava * (1) * FREQ_OF_NOTES(nota),...
            69 + 12 * log2(2^octava * (1 + DETUNE) * FREQ_OF_NOTES(nota)/440),...
            2^octava * (1 + DETUNE) * FREQ_OF_NOTES(nota));
    
    end
    
end
%% https://mixbutton.com/mixing-articles/music-note-to-frequency-chart/
f = 16.35:0.01:7902.13;
notaMidi = round(69 + 12 * log2(f / 440),0);
plot(f,notaMidi);grid;

    

