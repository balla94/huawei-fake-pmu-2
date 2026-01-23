#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// PSU VOLTAGE CONFIGURATION
// =============================================================================
// Values are in centivolts (real value * 100)
// Example: 3600 = 36.00V
#define PSU_VOLTAGE_MIN         3000    // 30.00V minimum
#define PSU_VOLTAGE_MAX         6000    // 60.00V maximum
#define PSU_VOLTAGE_DEFAULT     3600    // 36.00V default

// =============================================================================
// PSU CURRENT CONFIGURATION (per PSU)
// =============================================================================
// Values are in centiamps (real value * 100)
// Example: 5000 = 50.00A
#define PSU_CURRENT_MIN         100     // 1.00A minimum
#define PSU_CURRENT_MAX         5000    // 50.00A maximum
#define PSU_CURRENT_DEFAULT     100     // 1.00A default

// =============================================================================
// GROUP CURRENT CONFIGURATION (combined for all PSUs)
// =============================================================================
// Values are in centiamps (real value * 100)
// Example: 50000 = 500.00A (10 PSUs * 50A max each)
#define GROUP_CURRENT_MIN       100     // 1.00A minimum
#define GROUP_CURRENT_MAX       50000   // 500.00A maximum
#define GROUP_CURRENT_DEFAULT   100     // 1.00A default

// =============================================================================
// EEPROM CONFIGURATION
// =============================================================================
#define EEPROM_MAGIC            0xBADA  // Magic number to identify valid config
#define EEPROM_VERSION          1       // Version for future compatibility

// =============================================================================
// DEFAULT SETTINGS
// =============================================================================
#define DEFAULT_GROUP_MODE          false
#define DEFAULT_OUTPUT_ENABLED      false
#define DEFAULT_WAS_ON_BUS          false
#define DEFAULT_ALWAYS_ON_BACKLIGHT true

#endif // CONFIG_H
