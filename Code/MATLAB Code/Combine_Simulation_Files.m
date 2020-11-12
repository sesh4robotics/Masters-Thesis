MAT_File_name = "Name";                                                     %Name of .mat file without underscore or numbers
start_number = 0;                                                           %Starting Number for .mat files
end_number = 100;                                                           %Ending Number for .mat files
Name_of_Imported_Variable = "ScopeData3";                                   %Name of variable being imported

Time = [];
Data_Extracted = [];

for i = start_number:end_number
   
    number = sprintf('%d', i);
    filename = strcat(MAT_File_name, "_", number);
    Data_from_File = load(filename, '-mat');
    
    Time = [Time ; eval(join(["Data_from_File", Name_of_Imported_Variable, "time"], "."))];
    Data_Extracted = [Data_Extracted ; eval(join(["Data_from_File", Name_of_Imported_Variable, "signals", "values"], "."))];
    
end

figure;
plot(Time, Data_Extracted)

