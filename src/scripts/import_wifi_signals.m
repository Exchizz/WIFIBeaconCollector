files = dir('mmi_building_*_clean');

for file = files'
   file.name
   data =  dlmread(file.name,',',1,0);
   eval([file.name '= data']);

end