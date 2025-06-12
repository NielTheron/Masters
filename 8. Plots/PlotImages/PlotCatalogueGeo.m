function PlotCatalogueGeo(catalogue_geo, sample)

figure('Name','Cataloague');
scatter(catalogue_geo(2,:,sample),catalogue_geo(1,:,sample))

end

