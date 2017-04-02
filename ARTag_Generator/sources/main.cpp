#include <iostream>
#include "menu.h"
#include "ARTag.h"


int main(int argc, char *argv[]) {

	Menu menu;
	menu.printBanner();
	menu.askSize();
	menu.askMatrix();

	bool rotationOK = false;
	while (!rotationOK) {
		ARTag artag(menu.getSize(), menu.getMatrix());
		rotationOK = artag.checkRotations();
		if (rotationOK) {
			artag.generateImageFile();
			menu.printAndWait(artag.getIdentifier());
		}
	}

	return EXIT_SUCCESS;
}