rm record.rrf
rm ply/*
./sampleRetrieveData
./sampleExportPLY record.rrf
mv *.ply ply/
