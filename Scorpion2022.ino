/**************************************************************************
    Stars2021 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY;without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    See <https://www.gnu.org/licenses/>.
*/

#include "WOS_Config.h"
#include "WOS.h"
#include "DropTargets.h"
#include "Scorpion.h"
#include "SelfTestAndAudit.h"
#include "AudioHandler.h"
#include "LampAnimations.h"
#include <EEPROM.h>


// Wav Trigger defines have been moved to WOS_Config.h

#define USE_SCORE_OVERRIDES


#define SCORPION_MAJOR_VERSION  2023
#define SCORPION_MINOR_VERSION  1
#define DEBUG_MESSAGES  1

/*
 * Bugs & features
 * 
 * Drain during power up should eject ball and return to unstructured
 * 
 */


void PlaySoundEffect(unsigned int soundEffectNum);
void PlayBackgroundSong(unsigned int songNum);

/*********************************************************************

    Game specific code

*********************************************************************/

// MachineState
//  0 - Attract Mode
//  negative - self-test modes
//  positive - game play
char MachineState = 0;
boolean MachineStateChanged = true;
#define MACHINE_STATE_ATTRACT         0
#define MACHINE_STATE_INIT_GAMEPLAY   1
#define MACHINE_STATE_INIT_NEW_BALL   2
#define MACHINE_STATE_NORMAL_GAMEPLAY 4
#define MACHINE_STATE_COUNTDOWN_BONUS 99
#define MACHINE_STATE_BALL_OVER       100
#define MACHINE_STATE_MATCH_MODE      110
#define MACHINE_STATE_DIAGNOSTICS     120

#define MACHINE_STATE_ADJUST_FREEPLAY             -21
#define MACHINE_STATE_ADJUST_BALL_SAVE            -22
#define MACHINE_STATE_ADJUST_SOUND_SELECTOR       -23
#define MACHINE_STATE_ADJUST_MUSIC_VOLUME         -24
#define MACHINE_STATE_ADJUST_SFX_VOLUME           -25
#define MACHINE_STATE_ADJUST_CALLOUTS_VOLUME      -26
#define MACHINE_STATE_ADJUST_TOURNAMENT_SCORING   -27
#define MACHINE_STATE_ADJUST_TILT_WARNING         -28
#define MACHINE_STATE_ADJUST_AWARD_OVERRIDE       -29
#define MACHINE_STATE_ADJUST_BALLS_OVERRIDE       -30
#define MACHINE_STATE_ADJUST_SCROLLING_SCORES     -31
#define MACHINE_STATE_ADJUST_EXTRA_BALL_AWARD     -32
#define MACHINE_STATE_ADJUST_SPECIAL_AWARD        -33
#define MACHINE_STATE_ADJUST_CREDIT_RESET_HOLD_TIME -34
#define MACHINE_STATE_ADJUST_DONE                 -35

#define GAME_MODE_PRE_PLUNGE_CHOICE                 0
#define GAME_MODE_SKILL_SHOT                        1
#define GAME_MODE_BUILD_WEAPONS                     2
#define GAME_MODE_PRESENT_BLUE_LOCK                 3
#define GAME_MODE_ADD_LEFT_WEAPON                   4
#define GAME_MODE_PRESENT_GREEN_LOCK                5
#define GAME_MODE_ADD_RIGHT_WEAPON                  6
#define GAME_MODE_BATTLE_MULTIBALL_START            7
#define GAME_MODE_BATTLE_MULTIBALL                  8
#define GAME_MODE_BATTLE_FINISHING_SHOT             9
#define GAME_MODE_BATTLE_WON                        10
#define GAME_MODE_BATTLE_LOST                       11
#define GAME_MODE_DEFEND_MULTIBALL_START            12
#define GAME_MODE_DEFEND_MULTIBALL                  13
#define GAME_MODE_DEFEND_MULTIBALL_LOST             14
#define GAME_MODE_DEFEND_MULTIBALL_WON              15
#define GAME_MODE_DEFEND_MULTIBALL_WAIT_FOR_LOCK    16
#define GAME_MODE_CHASE_START                       17
#define GAME_MODE_CHASE_SPINNER                     18
#define GAME_MODE_CHASE_TOP_LANES                   19
#define GAME_MODE_CHASE_POP_BUMPERS                 20
#define GAME_MODE_CHASE_STANDUPS                    21
#define GAME_MODE_CHASE_LANE_TO_TOP                 22
#define GAME_MODE_CHASE_6_DROPS                     23
#define GAME_MODE_CHASE_5_DROPS                     24
#define GAME_MODE_CHASE_BULLSEYE                    25
#define GAME_MODE_CHASE_FINAL_SHOT                  26
#define GAME_MODE_CHASE_WON                         27
#define GAME_MODE_CHASE_LOST                        28
#define GAME_MODE_EGGS_START                        29
#define GAME_MODE_EGGS_IN_PROGRESS                  30
#define GAME_MODE_EGGS_WON                          31
#define GAME_MODE_EGGS_LOST                         32
#define GAME_MODE_VICTORY_START                     33
#define GAME_MODE_VICTORY                           34
#define GAME_MODE_VICTORY_END                       35
#define GAME_MODE_WAIT_TO_RESTART_BATTLE            36
#define GAME_MODE_RESTART_BATTLE                    37


#define EEPROM_BALL_SAVE_BYTE           100
#define EEPROM_FREE_PLAY_BYTE           101
#define EEPROM_SOUND_SELECTOR_BYTE      102
#define EEPROM_SKILL_SHOT_BYTE          103
#define EEPROM_TILT_WARNING_BYTE        104
#define EEPROM_AWARD_OVERRIDE_BYTE      105
#define EEPROM_BALLS_OVERRIDE_BYTE      106
#define EEPROM_TOURNAMENT_SCORING_BYTE  107
#define EEPROM_SFX_VOLUME_BYTE          108
#define EEPROM_MUSIC_VOLUME_BYTE        109
#define EEPROM_SCROLLING_SCORES_BYTE    110
#define EEPROM_CALLOUTS_VOLUME_BYTE     111
#define EEPROM_CRB_HOLD_TIME            118
#define EEPROM_EXTRA_BALL_SCORE_UL      140
#define EEPROM_SPECIAL_SCORE_UL         144


#define SOUND_EFFECT_NONE                     0
#define SOUND_EFFECT_BONUS_COUNT              1
#define SOUND_EFFECT_BONUS_COLLECT            2
#define SOUND_EFFECT_OUTLANE_UNLIT            4
#define SOUND_EFFECT_LOCK_BOUNCED             5
#define SOUND_EFFECT_OUTLANE_LIT              6
#define SOUND_EFFECT_BUMPER_HIT               7
#define SOUND_EFFECT_LOWER_BUMPER_HIT         8
//#define SOUND_EFFECT_WAITING_FOR_SKILL      9
//#define SOUND_EFFECT_ADD_CREDIT             10
#define SOUND_EFFECT_TOP_LANE_REPEAT          11
#define SOUND_EFFECT_TOP_LANE_NEW             12
#define SOUND_EFFECT_TOP_LANE_LEVEL_FINISHED  13
#define SOUND_EFFECT_SW_LETTER_AWARDED        14
#define SOUND_EFFECT_DROP_TARGET_HIT          15
#define SOUND_EFFECT_DROP_TARGET_RESET        16
#define SOUND_EFFECT_WEAPONS_SELECT_START     17
#define SOUND_EFFECT_BATTLE_MULTIBALL_START   18
#define SOUND_EFFECT_BALL_OVER                19
#define SOUND_EFFECT_GAME_OVER                20
#define SOUND_EFFECT_FRENZY_BUMPER_HIT        21
#define SOUND_EFFECT_STANDUP_HIT              22
#define SOUND_EFFECT_STANDUP_DUPLICATE        23
#define SOUND_EFFECT_STANDUPS_COMPLETE        24
#define SOUND_EFFECT_SKILL_SHOT               25
#define SOUND_EFFECT_TILT_WARNING             28
#define SOUND_EFFECT_MATCH_SPIN               30
#define SOUND_EFFECT_SPINNER                  32
#define SOUND_EFFECT_SPINNER_ALT              33
#define SOUND_EFFECT_SLING_SHOT               34
//#define SOUND_EFFECT_10PT_SWITCH            36
#define SOUND_EFFECT_BULLSEYE_UNLIT           37
#define SOUND_EFFECT_BULLSEYE_LIT             38
#define SOUND_EFFECT_LEFT_WEAPON_FINISHED     40
#define SOUND_EFFECT_RIGHT_WEAPON_FINISHED    41
#define SOUND_EFFECT_FLASH_FINISHED           42
#define SOUND_EFFECT_CAPTIVE_BALL_UNLIT       43
#define SOUND_EFFECT_BATTLE_ALARM             44
#define SOUND_EFFECT_BATTLE_LOST              45
#define SOUND_EFFECT_BATTLE_WON               46
#define SOUND_EFFECT_ENEMY_HIT                47
#define SOUND_EFFECT_ENEMY_RETALIATE          48
#define SOUND_EFFECT_ENEMY_CRITICAL_HIT       49
#define SOUND_EFFECT_AIR_ASSAULT              50
#define SOUND_EFFECT_ENEMY_SNEAK_ATTACK       51
#define SOUND_EFFECT_SMALL_EXPLOSION          52
#define SOUND_EFFECT_EGGS_HATCHING            53
#define SOUND_EFFECT_TILT                     61
#define SOUND_EFFECT_SCORE_TICK               67
#define SOUND_EFFECT_BONUS_TIMER_EXPLOSION    68
#define SOUND_EFFECT_BONUS_TIMER_PAYOUT       69
#define SOUND_EFFECT_CREATURE_SCREAM_LEFT_1   70
#define SOUND_EFFECT_CREATURE_SCREAM_LEFT_2   71
#define SOUND_EFFECT_CREATURE_SCREAM_LEFT_3   72
#define SOUND_EFFECT_CREATURE_SCREAM_LEFT_4   73
#define SOUND_EFFECT_CREATURE_SCREAM_LEFT_5   74
#define SOUND_EFFECT_CREATURE_SCREAM_RIGHT_1  75
#define SOUND_EFFECT_CREATURE_SCREAM_RIGHT_2  76
#define SOUND_EFFECT_CREATURE_SCREAM_RIGHT_3  77
#define SOUND_EFFECT_CREATURE_SCREAM_RIGHT_4  78
#define SOUND_EFFECT_CREATURE_SCREAM_RIGHT_5  79
#define SOUND_EFFECT_CREATURE_DYING_1         80
#define SOUND_EFFECT_CREATURE_DYING_2         81
#define SOUND_EFFECT_CREATURE_ANGRY_1         82
#define SOUND_EFFECT_CREATURE_ANGRY_2         83
#define SOUND_EFFECT_CREATURE_COMING          84
#define SOUND_EFFECT_DEFEND_POWERUP           85
#define SOUND_EFFECT_SCANNER_TICK             86
#define SOUND_EFFECT_MACHINE_START            87

#define SOUND_EFFECT_WAV_MANDATORY      100 
#define SOUND_EFFECT_COIN_DROP_1        100
#define SOUND_EFFECT_COIN_DROP_2        101
#define SOUND_EFFECT_COIN_DROP_3        102
#define SOUND_EFFECT_WIZARD_START_SAUCER  110
#define SOUND_EFFECT_WIZARD_FINAL_SHOT_1  111
#define SOUND_EFFECT_WIZARD_FINAL_SHOT_2  112

#define SOUND_EFFECT_SELF_TEST_MODE_START             132
#define SOUND_EFFECT_SELF_TEST_CPC_START              180
#define SOUND_EFFECT_SELF_TEST_AUDIO_OPTIONS_START    190

#define SOUND_EFFECT_BACKGROUND_SONG_1    500
#define NUM_BACKGROUND_SONGS              10
#define SOUND_EFFECT_BATTLE_SONG_1        525
#define NUM_BATTLE_SONGS                  6



/*
  Player.
  One.
  Two.
  Three. 
  Four.
  Launch When Ready.
  Use the Right Flipper to Select a weapons bank.
  Depth Charges.
  Cannons.
  Harpoons.
  Air Assault.
  Rail Guns.
  Torpedoes.
  Blue Laser.
  Green Laser.
  Less than.
  Twenty-five Percent.
  Fifty Percent.
  Seventy-five Percent.
  Ninety Percent.
  Ninety-five Percent.
  One-hundred Percent.
  Green lock ready.
  Blue lock ready.
  Select auxiliary weapons bank.
  Hold right flipper to.
  Use the right flipper to.
  Lock ball.
  Continue preparing weapons.
  Press the right flipper to.
  Complete.
  Prepared.
  Scorpion 3.
  Command.
  Commander.
  Ready for.
  Ready to.
  Engaging the creature.
  In.
  With.
  Battle.
  Pursuit.
  Capture.
  Ball.
  Locked.
  Green.
  Blue.
  returning to 1 ex playfield multiplier.
  2 ex playfield multiplier.
  3 ex playfield multiplier.
  4 ex playfield multiplier.
  5 ex playfield multiplier.
  Bonus multiplier increased.
  maximum bonus achieved.
  battle lost.
  expended.


  one player.
  two players.
  three players.
  four players.
  shoot again.
  extra ball.
  Jackpot.
  super jackpot.
  critical hit.
  creature is fleeing.
  Timers frozen.
  Timers resuming.
 * 
 */
// Game play status callouts
#define NUM_VOICE_NOTIFICATIONS                 183

#define SOUND_EFFECT_VP_VOICE_NOTIFICATIONS_START                 300
#define SOUND_EFFECT_VP_PLAYER                                    300
#define SOUND_EFFECT_VP_ONE                                       301
#define SOUND_EFFECT_VP_TWO                                       302
#define SOUND_EFFECT_VP_THREE                                     303
#define SOUND_EFFECT_VP_FOUR                                      304
#define SOUND_EFFECT_VP_LAUNCH_WHEN_READY                         305
#define SOUND_EFFECT_VP_USE_RIGHT_FLIPPER_FOR_WEAPONS_BANK        306
#define SOUND_EFFECT_VP_DEPTH_CHARGES                             307         
#define SOUND_EFFECT_VP_CANNONS                                   308   
#define SOUND_EFFECT_VP_HARPOONS                                  309    
#define SOUND_EFFECT_VP_AIR_ASSAULT                               310       
#define SOUND_EFFECT_VP_RAIL_GUNS                                 311     
#define SOUND_EFFECT_VP_TORPEDOES                                 312     
#define SOUND_EFFECT_VP_BLUE_LASER                                313      
#define SOUND_EFFECT_VP_GREEN_LASER                               314       
#define SOUND_EFFECT_VP_LESS_THAN                                 315     
#define SOUND_EFFECT_VP_25_PERCENT                                316      
#define SOUND_EFFECT_VP_50_PERCENT                                317      
#define SOUND_EFFECT_VP_75_PERCENT                                318      
#define SOUND_EFFECT_VP_90_PERCENT                                319      
#define SOUND_EFFECT_VP_95_PERCENT                                320      
#define SOUND_EFFECT_VP_100_PERCENT                               321       
#define SOUND_EFFECT_VP_JACKPOT                                   322
#define SOUND_EFFECT_VP_SUPER_JACKPOT                             323
#define SOUND_EFFECT_VP_CRITICAL_HIT                              324
#define SOUND_EFFECT_VP_CREATURE_FLEEING                          325
#define SOUND_EFFECT_VP_TIMERS_FROZEN                             326

#define SOUND_EFFECT_VP_ADD_PLAYER_1        327
#define SOUND_EFFECT_VP_ADD_PLAYER_2        (SOUND_EFFECT_VP_ADD_PLAYER_1+1)
#define SOUND_EFFECT_VP_ADD_PLAYER_3        (SOUND_EFFECT_VP_ADD_PLAYER_1+2)
#define SOUND_EFFECT_VP_ADD_PLAYER_4        (SOUND_EFFECT_VP_ADD_PLAYER_1+3)
#define SOUND_EFFECT_VP_SHOOT_AGAIN         331

#define SOUND_EFFECT_VP_EXTRA_BALL                                332
#define SOUND_EFFECT_VP_TIMERS_RESUMING                           333
#define SOUND_EFFECT_VP_BALL_MISSING                              334
#define SOUND_EFFECT_VP_SPACER                                    335
#define SOUND_EFFECT_VP_GREEN_LOCK_READY                          336
#define SOUND_EFFECT_VP_BLUE_LOCK_READY                           337
#define SOUND_EFFECT_VP_SELECT_AUX_WEAPONS_BANK                   338
#define SOUND_EFFECT_VP_HOLD_RIGHT_FLIPPER                        339
#define SOUND_EFFECT_VP_USE_RIGHT_FLIPPER                         340
#define SOUND_EFFECT_VP_LOCK_BALL                                 341
#define SOUND_EFFECT_VP_CONTINUE_PREPARING_WEAPONS                342
#define SOUND_EFFECT_VP_PRESS_RIGHT_FLIPPER                       343
#define SOUND_EFFECT_VP_COMPLETE                                  344
#define SOUND_EFFECT_VP_PREPARED                                  345
#define SOUND_EFFECT_VP_SCORPION_3                                346
#define SOUND_EFFECT_VP_COMMAND                                   347
#define SOUND_EFFECT_VP_COMMANDER                                 348
#define SOUND_EFFECT_VP_READY_FOR                                 349
#define SOUND_EFFECT_VP_READY_TO                                  350
#define SOUND_EFFECT_VP_ENGAGING_THE_CREATURE                     351
#define SOUND_EFFECT_VP_IN                                        352
#define SOUND_EFFECT_VP_WITH                                      353
#define SOUND_EFFECT_VP_BATTLE                                    354
#define SOUND_EFFECT_VP_PURSUIT                                   355
#define SOUND_EFFECT_VP_CAPTURE                                   356
#define SOUND_EFFECT_VP_BALL                                      357
#define SOUND_EFFECT_VP_LOCKED                                    358
#define SOUND_EFFECT_VP_GREEN                                     359
#define SOUND_EFFECT_VP_BLUE                                      360
#define SOUND_EFFECT_VP_RETURN_TO_1X                              361
#define SOUND_EFFECT_VP_2X                                        362
#define SOUND_EFFECT_VP_3X                                        363
#define SOUND_EFFECT_VP_4X                                        364
#define SOUND_EFFECT_VP_5X                                        365
#define SOUND_EFFECT_VP_BONUS_X_INCREASED                         366
#define SOUND_EFFECT_VP_MAX_BONUS                                 367
#define SOUND_EFFECT_VP_BATTLE_LOST                               368
#define SOUND_EFFECT_VP_EXPENDED                                  369
#define SOUND_EFFECT_VP_HOLD_RIGHT_DISABLE_LOCK                   370
#define SOUND_EFFECT_VP_SELECT_WEAOPNS_RIGHT_FLIPPER              371
#define SOUND_EFFECT_VP_ALREADY_ACTIVE                            372
#define SOUND_EFFECT_VP_SELECT_WEAPON_INTRO                       373
#define SOUND_EFFECT_VP_NEST_IN_1                                 374
#define SOUND_EFFECT_VP_NEST_IN_2                                 375
#define SOUND_EFFECT_VP_NEST_IN_3                                 376
#define SOUND_EFFECT_VP_NEST_IN_4                                 377
#define SOUND_EFFECT_VP_NEST_IN_5                                 378
#define SOUND_EFFECT_VP_NEST_IN_6                                 379
#define SOUND_EFFECT_VP_EGGS_HATCHING                             380
#define SOUND_EFFECT_VP_HATCHLINGS_MOVING                         381
#define SOUND_EFFECT_VP_ON_THE_RUN                                382
#define SOUND_EFFECT_VP_LOCK_UP_WEAPONS                           383
#define SOUND_EFFECT_VP_PARK_WEAPON                               384
#define SOUND_EFFECT_VP_GETTING_INTO_WEAPONS                      385
#define SOUND_EFFECT_VP_WEAPONS_UNDER_ATTACK                      386
#define SOUND_EFFECT_VP_WEAPONS_COMPROMISED                       387
#define SOUND_EFFECT_VP_RIGHT_SIDE_DONE                           388
#define SOUND_EFFECT_VP_LEFT_SIDE_DONE                            389
#define SOUND_EFFECT_VP_WOKE_THE_CREATURE_1                       390
#define SOUND_EFFECT_VP_WOKE_THE_CREATURE_2                       391
#define SOUND_EFFECT_VP_WOKE_THE_CREATURE_3                       392
#define SOUND_EFFECT_VP_WOKE_THE_CREATURE_4                       393
#define SOUND_EFFECT_VP_WOKE_THE_CREATURE_5                       394
#define SOUND_EFFECT_VP_FULL_CHARGE                               395
#define SOUND_EFFECT_VP_ACTIVATE_THE_WEAPON                       396
#define SOUND_EFFECT_VP_ACTIVATE_TOP_LANE                         397
#define SOUND_EFFECT_VP_ACTIVATE_POP                              398
#define SOUND_EFFECT_VP_ACTIVATE_6_DROP                           399
#define SOUND_EFFECT_VP_ACTIVATE_STANDUP                          400
#define SOUND_EFFECT_VP_ACTIVATE_BULLSEYE                         401
#define SOUND_EFFECT_VP_ACTIVATE_5_DROP                           402
#define SOUND_EFFECT_VP_ACTIVATE_BLUE                             403
#define SOUND_EFFECT_VP_ACTIVATE_GREEN                            404
#define SOUND_EFFECT_VP_CHARGE_OR_LOCK                            405
#define SOUND_EFFECT_VP_CHARGE_FIRST_BALL                         406
#define SOUND_EFFECT_VP_LOCK_SECOND                               407
#define SOUND_EFFECT_VP_CHARGER_OVERLOAD                          408
#define SOUND_EFFECT_VP_GOING_TO_LAY_EGGS                         409
#define SOUND_EFFECT_VP_SENSORS_PICKING_UP_EGGS                   410
#define SOUND_EFFECT_VP_ZONE_1                                    411
#define SOUND_EFFECT_VP_ZONE_2                                    412
#define SOUND_EFFECT_VP_ZONE_3                                    413
#define SOUND_EFFECT_VP_ZONE_4                                    414
#define SOUND_EFFECT_VP_ZONE_5                                    415
#define SOUND_EFFECT_VP_ZONE_6                                    416
#define SOUND_EFFECT_VP_GET_OVER_THERE_BEFORE_HATCH               417
#define SOUND_EFFECT_VP_HATCHING_IN_PROGRESS                      418
#define SOUND_EFFECT_VP_HATCHLINGS_ON_MOVE                        419
#define SOUND_EFFECT_VP_HATCHLINGS_3_BLIPS                        420
#define SOUND_EFFECT_VP_HATCHLINGS_LOOSE                          421
#define SOUND_EFFECT_VP_HATCHLINGS_TAKE_OUT_WEAPONS               422
#define SOUND_EFFECT_VP_NICE_WORK_DEFENSES_HELD                   423
#define SOUND_EFFECT_VP_CHASE_IT_GENERIC                          424
#define SOUND_EFFECT_VP_SENSORS_GET_ON_IT                         425
#define SOUND_EFFECT_VP_CHASE_THE_THINGS                          426
#define SOUND_EFFECT_VP_CHASE_BULLSEYES                           427
#define SOUND_EFFECT_VP_ALMOST_DONE_CHASE_LOCK_IT                 428
#define SOUND_EFFECT_VP_BREATHING_ROOM_BUILD                      429
#define SOUND_EFFECT_VP_SAUCER_PICK_AND_BUILD                     430
#define SOUND_EFFECT_VP_GET_WEAPONS_TO_100                        431
#define SOUND_EFFECT_VP_ONE_WEAPON_DONE                           432
#define SOUND_EFFECT_VP_WEAPONS_ARE_READY                         433
#define SOUND_EFFECT_VP_LOCK_IT_UP                                434
#define SOUND_EFFECT_VP_EGGS_ERADICATED                           435
#define SOUND_EFFECT_VP_LAST_HATCHLING_DONE                       436
#define SOUND_EFFECT_VP_DEFENSE_START                             437
#define SOUND_EFFECT_VP_REBUILD_DEFENSES                          438
#define SOUND_EFFECT_VP_CHASE_FAILED                              439
#define SOUND_EFFECT_VP_LIMITED_TIME_TO_FINISH_CHASE              440
#define SOUND_EFFECT_VP_RIGHT_FLIPPER_SCAN_CHASE                  441
#define SOUND_EFFECT_VP_BATTLE_LOST_REGROUP                       442
#define SOUND_EFFECT_VP_VICTORY_START                             443
#define SOUND_EFFECT_VP_VICTORY_INSTRUCTIONS                      444
#define SOUND_EFFECT_VP_CHASE_IT_QUICK                            445
#define SOUND_EFFECT_VP_SPINNER_STARTS_CHASE_1                    446
#define SOUND_EFFECT_VP_SPINNER_STARTS_CHASE_2                    447
#define SOUND_EFFECT_VP_WERE_CHASING                              448
#define SOUND_EFFECT_VP_NO_CHASE_TODAY                            449
#define SOUND_EFFECT_VP_PLUNGE_SECOND_DEFEND                      450
#define SOUND_EFFECT_VP_WEAPONS_COOKED                            451
#define SOUND_EFFECT_VP_WEAPON_ACTIVATED                          452
#define SOUND_EFFECT_VP_CREATURE_OUT_OF_RANGE                     453
#define SOUND_EFFECT_VP_WON_THE_BATTLE                            454
#define SOUND_EFFECT_VP_LEFT_WEAPON_ACTIVATED                     455
#define SOUND_EFFECT_VP_RIGHT_WEAPON_ACTIVATED                    456
#define SOUND_EFFECT_VP_HOLDOVER_MULTIBALL_TIMER                  457
#define SOUND_EFFECT_VP_HOLDOVER_BONUS                            458
#define SOUND_EFFECT_VP_HOLDOVER_BONUS_X                          459
#define SOUND_EFFECT_VP_JACKPOT_ALT_1                             460
#define SOUND_EFFECT_VP_JACKPOT_ALT_2                             461
#define SOUND_EFFECT_VP_SUPER_JACKPOT_ALT_1                       462
#define SOUND_EFFECT_VP_SUPER_JACKPOT_ALT_2                       463
#define SOUND_EFFECT_VP_FAILED_TO_DEFEND_CREATURE_MOVING          464
#define SOUND_EFFECT_VP_BATTLE_INSTRUCTIONS                       465
#define SOUND_EFFECT_VP_BATTLE_LEFT_SIDE_READY                    466
#define SOUND_EFFECT_VP_BATTLE_RIGHT_SIDE_READY                   467
#define SOUND_EFFECT_VP_VICTORY_DEPTH_CHARGES_READY               468
#define SOUND_EFFECT_VP_VICTORY_TORPEDOES_READY                   469
#define SOUND_EFFECT_VP_GOOD_ENOUGH_FOR_NOW                       470
#define SOUND_EFFECT_VP_WEAPONS_TOASTED                           471
#define SOUND_EFFECT_VP_LOWER_POP_HOLDOVER                        472
#define SOUND_EFFECT_VP_RIGHT_POP_HOLDOVER                        473
#define SOUND_EFFECT_VP_UPPER_POP_HOLDOVER                        474
#define SOUND_EFFECT_VP_VICTORY_END_NO_JACKPOT                    475
#define SOUND_EFFECT_VP_VICTORY_DONE_EVERYTHING                   476
#define SOUND_EFFECT_VP_DEFEND_LOST_100                           477
#define SOUND_EFFECT_VP_DEFEND_LOST_DURING_POWERUP                478
#define SOUND_EFFECT_VP_DEFEND_SECOND_TRY                         479
#define SOUND_EFFECT_VP_HIT_SAUCER_TO_RESTART_BATTLE              480
#define SOUND_EFFECT_VP_PLUNGE_TO_RESTART                         481
#define SOUND_EFFECT_VP_FIVE                                      482

#define SOUND_EFFECT_AP_CRB_OPTION_1    211
#define SOUND_EFFECT_AP_CRB_OPTION_99   210


#define MAX_DISPLAY_BONUS     19
#define TILT_WARNING_DEBOUNCE_TIME      1000

/*
unsigned short VoicePromptLengths[NUM_VOICE_NOTIFICATIONS] = {
  414, 448, 443, 553, 506, 1165, 2740, 1144, 696, 820,
  829, 1011, 938, 875, 926, 723, 1320, 1129, 1553, 1129,
  1439, 1320, 689, 1141, 987, 1279, 1097, 766, 953, 1069,
  1036, 948, 904, 1196, 860, 342, 1141, 1099, 2046, 1450,
  1604, 832, 1828, 1571, 717, 687, 1135, 666, 654, 814, 
  775, 1259, 418, 384, 484, 624, 566, 457, 524, 533,
  433, 2528, 1928, 1971, 2040, 1980, 1922, 1707, 975, 754,
  2096, 2266, 1075, 3825, 3756, 3234, 2921, 3373, 3617, 3025,
  2469, 4382, 2365, 2156, 2921, 2786, 1695, 2907, 3413, 3228,
  1939, 1463, 3146, 3390, 2403, 1599, 1300, 2407, 2449, 3388, 
  2518, 2787, 3290, 2753, 2720, 3928, 2518, 1242, 2216, 3135,
  1962, 813, 871, 917, 929, 987, 1010, 1521, 1219, 1591,
  4447, 2705, 1556, 2083, 1611, 3144, 1690, 3852, 1533, 3616,
  2673, 3459, 2122, 3196, 817, 2706, 2426, 2961, 2228
};
*/
/*********************************************************************

    Machine state and options

*********************************************************************/
byte Credits = 0;
byte SoundSelector = 3;
byte BallSaveNumSeconds = 0;
byte MaximumCredits = 40;
byte BallsPerGame = 3;
byte ScoreAwardReplay = 0;
byte MusicVolume = 6;
byte SoundEffectsVolume = 8;
byte CalloutsVolume = 10;
byte ChuteCoinsInProgress[3];
byte TotalBallsLoaded = 2;
byte TimeRequiredToResetGame = 1;
byte NumberOfBallsInPlay = 0;
byte NumberOfBallsLocked = 0;
byte LampType = 0;
boolean FreePlayMode = false;
boolean HighScoreReplay = true;
boolean MatchFeature = true;
boolean TournamentScoring = false;
boolean ScrollingScores = true;
unsigned long ExtraBallValue = 0;
unsigned long SpecialValue = 0;
unsigned long CurrentTime = 0;
unsigned long HighScore = 0;
unsigned long AwardScores[3];
unsigned long CreditResetPressStarted = 0;

AudioHandler Audio;



/*********************************************************************

    Game State

*********************************************************************/
byte CurrentPlayer = 0;
byte CurrentBallInPlay = 1;
byte CurrentNumPlayers = 0;
byte BonusLeft[4];
byte BonusRight[4];
byte BonusX[4];
byte GameMode = GAME_MODE_SKILL_SHOT;
byte MaxTiltWarnings = 2;
byte NumTiltWarnings = 0;
byte CurrentAchievements[4];
byte LastSwitchHit;

boolean SamePlayerShootsAgain = false;
boolean BallSaveUsed = false;
boolean ExtraBallCollected = false;
boolean SpecialCollected = false;
boolean TimersPaused = true;
boolean AllowResetAfterBallOne = true;

unsigned long CurrentScores[4];
unsigned long BallFirstSwitchHitTime = 0;
unsigned long BallTimeInTrough = 0;
unsigned long GameModeStartTime = 0;
unsigned long GameModeEndTime = 0;
unsigned long LastTiltWarningTime = 0;
unsigned long ScoreAdditionAnimation;
unsigned long ScoreAdditionAnimationStartTime;
unsigned long LastRemainingAnimatedScoreShown;
unsigned long PlayfieldMultiplier;
unsigned long TotalScoreMultiplier;
unsigned long LastTimeThroughLoop;
unsigned long LastSwitchHitTime;
unsigned long BallSaveEndTime;

#define BALL_SAVE_GRACE_PERIOD  4000

/*********************************************************************

    Game Specific State Variables

*********************************************************************/
byte TotalPopHits[4][3];
byte HoldoverAwards[4];
byte SpinnerMaxGoal = 250;
byte TopLaneSkillShot;
byte Num3DropsCleared[4];
byte Num5DropsCleared[4];
byte CombosAchieved[4];
byte IdleMode;
byte CombosToFinishGoal = 6;
byte SkillShotLane;
byte SkillShotLaneHit;
byte CreatureHitsLeft;
byte CreatureHitsRight;
//#define NUM_BALL_SEARCH_SOLENOIDS   5
//byte BallSearchSolenoidToTry;
//byte BallSearchSols[NUM_BALL_SEARCH_SOLENOIDS] = {SOL_BOTTOM_POP, SOL_RIGHT_POP, SOL_TOP_POP, SOL_RIGHT_SLING, SOL_LEFT_SLING};

byte CurrentWeaponsEngaged[4];// this is a bitmask
byte CurrentWeaponSelected;// this is an index starting at 1 (0=no weapon)
byte WeaponsStatus[4][8];
byte TopLaneStatus[4];
byte TopLaneLevel[4];
byte StandupStatus[4];
byte StandupLevel[4];
byte MachineLocks;
byte LockStatus[4];
byte Drop5Clears[4];
byte Drop6Clears[4];
byte LastWeaponChanged;
byte LaneToTopValue[4];
byte CreatureDefense;

byte StoriesEngaged[4];
byte StoriesFinished[4];

boolean IdleModeOn = true;
boolean RightFlipperHeld = false;
boolean RightFlipperHeldLong = false;
boolean SuperSkillShotReady = false;
boolean WeaponChangedOnce = false;
boolean LeftSaucerLastState = false;
boolean RightSaucerLastState = false; 
boolean CreatureHitsLeftDecreasing = false;
boolean CreatureHitsRightDecreasing = false;
boolean LeftVictoryReady = false;
boolean RightVictoryReady = false;
boolean DoubleTimerBonus[4];

unsigned long BonusXAnimationStart;
unsigned long LastSpinnerHit;

unsigned long TopPopLastHit;
unsigned long BottomPopLastHit;
unsigned long RightPopLastHit;
unsigned long PlayfieldMultiplierExpiration;
unsigned long SpinnerFrenzyEndTime;
unsigned long BallSearchNextSolenoidTime;
//unsigned long BallSearchSolenoidFireTime[NUM_BALL_SEARCH_SOLENOIDS];
unsigned long LeftSaucerKicked = 0;
unsigned long RightSaucerKicked = 0;
unsigned long BallRampKicked = 0;
unsigned long RightFlipperDownTime = 0;
unsigned long TicksCountedTowardsStatus;
unsigned long BonusLeftChanged;
unsigned long BonusRightChanged;
unsigned long TopLanesCompletedTime;
unsigned long StandupsCompletedTime;
unsigned long WeaponsStatusShownTime;
unsigned long SkillShotLaneHitTime;
unsigned long LastRightPopHit;
unsigned long LastUpperPopHit;
unsigned long LastLowerPopHit;
unsigned long WeaponsProgressChangedTime;
unsigned long LastTopLaneHit[3];
unsigned long DefendLeftPowerUpStart;
unsigned long DefendRightPowerUpStart;
unsigned long DefenseBonus;
unsigned long LaneToTopLastHit;
unsigned long BuildWeaponsLastModeHit;
unsigned long HatchlingsProgress;
unsigned long LastInlaneHit[4];
unsigned long LastOutlaneHit;
unsigned long BonusTimeAccumulated[4];
unsigned long LeftSaucerDownTime = 0;
unsigned long LeftSaucerUpTime = 0;
unsigned long RightSaucerDownTime = 0;
unsigned long RightSaucerUpTime = 0;
unsigned long ChaseLegBonus = 0;
unsigned long LeftWeaponChangedTime = 0;
unsigned long RightWeaponChangedTime = 0;
unsigned long LastCreatureHitLeftTime = 0;
unsigned long LastCreatureHitRightTime = 0;
unsigned long LeftJackpotTime = 0;
unsigned long RightJackpotTime = 0;



// Combo tracking variables

DropTargetBank DropTargets6(6, 2, DROP_TARGET_TYPE_WILLIAMS_1, 50);
DropTargetBank DropTargets5(5, 2, DROP_TARGET_TYPE_WILLIAMS_1, 50);

#define HOLDOVER_BONUS                0x01 // right pop, level 2
#define HOLDOVER_BONUS_X              0x02 // top pop, level 3
#define HOLDOVER_MULTIBALL_TIMER      0x04 // lower pop & spinner, level 1
#define HOLDOVER_DOUBLE_TIMERS        0x08
#define POP_HITS_FOR_HOLDOVER         10

#define EGG_ERADICATION_BONUS         100000
#define EGGS_IDLE_TIME                20000
#define EGG_DEVELOPMENT_TIME          5000
#define HATCHLING_SPREAD_TIME         7000
#define HATCHLING_ZONE_1              0x00000001
#define HATCHLING_ZONE_2              0x00000002
#define HATCHLING_ZONE_3              0x00000004
#define HATCHLING_ZONE_4              0x00000008
#define HATCHLING_ZONE_5              0x00000010
#define HATCHLING_ZONE_6              0x00000020
#define HATCHLING_SPINNER             0x00000040
#define HATCHLING_UPPER_POP           0x00000080
#define HATCHLING_RIGHT_POP           0x00000100
#define HATCHLING_POP_BUMPERS         0x000001C0
#define HATCHLING_10K                 0x00000200
#define HATCHLING_20K                 0x00000400
#define HATCHLING_30K                 0x00000800
#define HATCHLING_50K                 0x00001000
#define HATCHLING_SPECIAL             0x00002000
#define HATCHLING_6DROP_BANK          0x00003E00
#define HATCHLING_LITES_OUT           0x00004000
#define HATCHLING_50K_RIGHT           0x00008000
#define HATCHLING_EXTRA               0x00010000
#define HATCHLING_SPECIAL_RIGHT       0x00020000
#define HATCHLING_LANE_TO_TOP         0x00038000
#define HATCHLING_ALL                 0x0003FFC0


#define STORY_DEFEND                  0x01
#define STORY_ATTACK                  0x02
#define STORY_RESTART                 0x04
#define STORY_CHASE                   0x08
#define STORY_EGGS                    0x10
#define ALL_BASIC_STORIES             0x1B
#define STORY_VICTORY                 0x20

#define WEAPONS_MODE_DEPTH_CHARGES    0x01  // 3 drops
#define WEAPONS_MODE_CANNONS          0x02  // Pops and spinner
#define WEAPONS_MODE_HARPOONS         0x04  // 1-2-3 stand ups
#define WEAPONS_MODE_AIR_ASSAULT      0x08  // Upper lane + 4-5-6 lanes
#define WEAPONS_MODE_RAIL_GUNS        0x10  // Bullseye targets
#define WEAPONS_MODE_TORPEDOES        0x20  // 5 drops
#define WEAPONS_MODE_BLUE_LASER       0x40  // Left saucer
#define WEAPONS_MODE_GREEN_LASER      0x80  // Right saucer

#define WEAPON_INDEX_DEPTH_CHARGES    1
#define WEAPON_INDEX_CANNONS          2
#define WEAPON_INDEX_HARPOONS         3
#define WEAPON_INDEX_AIR_ASSAULT      4
#define WEAPON_INDEX_RAIL_GUNS        5
#define WEAPON_INDEX_TORPEDOES        6
#define WEAPON_INDEX_BLUE_LASER       7
#define WEAPON_INDEX_GREEN_LASER      8

#define WEAPON_ADVANCE_DEPTH_CHARGES    5
#define WEAPON_ADVANCE_CANNONS          2
#define WEAPON_ADVANCE_HARPOONS         10
#define WEAPON_ADVANCE_AIR_ASSAULT      20
#define WEAPON_ADVANCE_RAIL_GUNS        50
#define WEAPON_ADVANCE_TORPEDOES        10
#define WEAPON_ADVANCE_BLUE_LASER       50
#define WEAPON_ADVANCE_GREEN_LASER      50

#define LOCK_STATUS_BLUE_READY          0x01
#define LOCK_STATUS_GREEN_READY         0x02
#define LOCK_STATUS_BLUE_LOCKED         0x04
#define LOCK_STATUS_GREEN_LOCKED        0x08

#define COMBO_AVAILABLE_TIME              3500
#define COMBO_LEFT_TO_RIGHT_ALLEY_PASS    0
#define COMBO_RIGHT_TO_LEFT_ALLEY_PASS    1
#define COMBO_LEFT_TO_RIGHT_SPINNER       2
#define COMBO_RIGHT_TO_LEFT_SPINNER       3
#define COMBO_LEFT_TO_BULLSEYE            4
#define COMBO_RIGHT_TO_CAPTIVE            5
#define COMBO_AWARD               10000
#define COMBOS_COMPLETE_AWARD     75000

#define IDLE_MODE_NONE                  0
#define IDLE_MODE_BALL_SEARCH           9

#define SKILL_SHOT_AWARD          25000
#define SUPER_SKILL_SHOT_AWARD    50000

#define JACKPOT_AWARD             150000
#define SUPER_JACKPOT_AWARD       600000
#define VICTORY_JACKPOT_VALUE     250000

#define POP_BUMPER_DEBOUNCE_TIME    200
#define TOP_LANE_DEBOUNCE_TIME      150
#define OUT_LANE_DEBOUNCE_TIME      150
#define IN_LANE_DEBOUNCE_TIME       200

/******************************************************
 * 
 * Adjustments Serialization
 * 
 */


void ReadStoredParameters() {
   for (byte count=0;count<3;count++) {
    ChuteCoinsInProgress[count] = 0;
  }
 
  HighScore = WOS_ReadULFromEEProm(WOS_HIGHSCORE_EEPROM_START_BYTE, 10000);
  Credits = WOS_ReadByteFromEEProm(WOS_CREDITS_EEPROM_BYTE);
  if (Credits > MaximumCredits) Credits = MaximumCredits;

  ReadSetting(EEPROM_FREE_PLAY_BYTE, 0);
  FreePlayMode = (EEPROM.read(EEPROM_FREE_PLAY_BYTE)) ? true : false;

  BallSaveNumSeconds = ReadSetting(EEPROM_BALL_SAVE_BYTE, 15);
  if (BallSaveNumSeconds > 20) BallSaveNumSeconds = 20;

  SoundSelector = ReadSetting(EEPROM_SOUND_SELECTOR_BYTE, 3);
  if (SoundSelector > 8) SoundSelector = 3;

  MusicVolume = ReadSetting(EEPROM_MUSIC_VOLUME_BYTE, 10);
  if (MusicVolume>10) MusicVolume = 10;

  SoundEffectsVolume = ReadSetting(EEPROM_SFX_VOLUME_BYTE, 10);
  if (SoundEffectsVolume>10) SoundEffectsVolume = 10;

  CalloutsVolume = ReadSetting(EEPROM_CALLOUTS_VOLUME_BYTE, 10);
  if (CalloutsVolume>10) CalloutsVolume = 10;

  Audio.SetMusicVolume(MusicVolume);
  Audio.SetSoundFXVolume(SoundEffectsVolume);
  Audio.SetNotificationsVolume(CalloutsVolume);

  TournamentScoring = (ReadSetting(EEPROM_TOURNAMENT_SCORING_BYTE, 0)) ? true : false;

  MaxTiltWarnings = ReadSetting(EEPROM_TILT_WARNING_BYTE, 2);
  if (MaxTiltWarnings > 2) MaxTiltWarnings = 2;

  TimeRequiredToResetGame = ReadSetting(EEPROM_CRB_HOLD_TIME, 1);
  if (TimeRequiredToResetGame>3 && TimeRequiredToResetGame!=99) TimeRequiredToResetGame = 1;

  byte awardOverride = ReadSetting(EEPROM_AWARD_OVERRIDE_BYTE, 99);
  if (awardOverride != 99) {
    ScoreAwardReplay = awardOverride;
  }

  byte ballsOverride = ReadSetting(EEPROM_BALLS_OVERRIDE_BYTE, 99);
  if (ballsOverride == 3 || ballsOverride == 5) {
    BallsPerGame = ballsOverride;
  } else {
    if (ballsOverride != 99) EEPROM.write(EEPROM_BALLS_OVERRIDE_BYTE, 99);
  }

  ScrollingScores = (ReadSetting(EEPROM_SCROLLING_SCORES_BYTE, 1)) ? true : false;

  ExtraBallValue = WOS_ReadULFromEEProm(EEPROM_EXTRA_BALL_SCORE_UL);
  if (ExtraBallValue % 1000 || ExtraBallValue > 100000) ExtraBallValue = 20000;

  SpecialValue = WOS_ReadULFromEEProm(EEPROM_SPECIAL_SCORE_UL);
  if (SpecialValue % 1000 || SpecialValue > 100000) SpecialValue = 40000;

  AwardScores[0] = WOS_ReadULFromEEProm(WOS_AWARD_SCORE_1_EEPROM_START_BYTE);
  AwardScores[1] = WOS_ReadULFromEEProm(WOS_AWARD_SCORE_2_EEPROM_START_BYTE);
  AwardScores[2] = WOS_ReadULFromEEProm(WOS_AWARD_SCORE_3_EEPROM_START_BYTE);

}


void MoveBallFromOutholeToRamp(boolean sawSwitch = false) {
  if (WOS_ReadSingleSwitchState(SW_OUTHOLE) || sawSwitch) {
    if (BallRampKicked==0 || CurrentTime>(BallRampKicked+1000)) {
      WOS_PushToSolenoidStack(SOL_OUTHOLE, 16, true);
      BallRampKicked = CurrentTime;
    }
  }
  
}


void setup() {

  // Set up the chips and interrupts
  WOS_InitializeMPU(/*SW_CREDIT_RESET*/);
  if (WOS_DiagnosticModeRequested()) {
    MachineState = MACHINE_STATE_DIAGNOSTICS;
  }
  if (DEBUG_MESSAGES) {
    Serial.begin(57600);
    Serial.write("Done with MPU Initialization\n");

//    char buf[64]; 
//    sprintf(buf, "top Machine State = %d\n", (int)MachineState);
//    Serial.write(buf);
  }
  WOS_DisableSolenoidStack();
  WOS_SetDisableFlippers(true);

  // Read parameters from EEProm
  ReadStoredParameters();
  WOS_SetCoinLockout((Credits >= MaximumCredits) ? true : false);

  CurrentScores[0] = SCORPION_MAJOR_VERSION;
  CurrentScores[1] = SCORPION_MINOR_VERSION;
  CurrentScores[2] = WOS_MAJOR_VERSION;
  CurrentScores[3] = WOS_MINOR_VERSION;

  CurrentAchievements[0] = 0;
  CurrentAchievements[1] = 0;
  CurrentAchievements[2] = 0;
  CurrentAchievements[3] = 0;

  DropTargets6.DefineSwitch(0, SW_LEFT_3_DROP_1);
  DropTargets6.DefineSwitch(1, SW_LEFT_3_DROP_2);
  DropTargets6.DefineSwitch(2, SW_LEFT_3_DROP_3);
  DropTargets6.DefineSwitch(3, SW_RIGHT_3_DROP_1);
  DropTargets6.DefineSwitch(4, SW_RIGHT_3_DROP_2);
  DropTargets6.DefineSwitch(5, SW_RIGHT_3_DROP_3);
  DropTargets6.AddAllTargetsSwitch(SW_3_DROPS_COMPLETE);
  DropTargets6.DefineResetSolenoid(0, SOL_3_DROP_LEFT_RESET);
  DropTargets6.DefineResetSolenoid(1, SOL_3_DROP_RIGHT_RESET);

  DropTargets5.DefineSwitch(0, SW_5_DROP_1);
  DropTargets5.DefineSwitch(1, SW_5_DROP_2);
  DropTargets5.DefineSwitch(2, SW_5_DROP_3);
  DropTargets5.DefineSwitch(3, SW_5_DROP_4);
  DropTargets5.DefineSwitch(4, SW_5_DROP_5);
  DropTargets5.AddAllTargetsSwitch(SW_5_DROP_COMPLETE);
  DropTargets5.DefineResetSolenoid(0, SOL_5_DROP_LEFT_RESET);
  DropTargets5.DefineResetSolenoid(1, SOL_5_DROP_RIGHT_RESET);

  if (DEBUG_MESSAGES) {
    char buf[100]; 
    sprintf(buf, "bottom of debug Machine State = %d\n", (int)MachineState);
    Serial.write(buf);
  }

/*
  Audio.InitDevices(AUDIO_PLAY_TYPE_WAV_TRIGGER | AUDIO_PLAY_TYPE_ORIGINAL_SOUNDS);
  Audio.StopAllAudio();
  delay(10);
  Audio.SetMusicDuckingGain(12);
  Audio.PlaySound(SOUND_EFFECT_MACHINE_START, AUDIO_PLAY_TYPE_WAV_TRIGGER);    
*/
  delay(1000);
  if (DEBUG_MESSAGES) {
    char buf[100]; 
    sprintf(buf, "real bottom of debug Machine State = %d\n", (int)MachineState);
    Serial.write(buf);
  }

}




byte ReadSetting(byte setting, byte defaultValue) {
  byte value = EEPROM.read(setting);
  if (value == 0xFF) {
    EEPROM.write(setting, defaultValue);
    return defaultValue;
  }
  return value;
}


// This function is useful for checking the status of drop target switches
byte CheckSequentialSwitches(byte startingSwitch, byte numSwitches) {
  byte returnSwitches = 0;
  for (byte count = 0;count < numSwitches;count++) {
    returnSwitches |= (WOS_ReadSingleSwitchState(startingSwitch + count) << count);
  }
  return returnSwitches;
}


////////////////////////////////////////////////////////////////////////////
//
//  Lamp Management functions
//
////////////////////////////////////////////////////////////////////////////

void ShowTopLaneLamps() {

  if (StoriesEngaged[CurrentPlayer]==STORY_EGGS) {
    byte lampPhase = ((CurrentTime/100)%20);
    boolean lampOn = false;
    if (lampPhase==0 || lampPhase==3 || lampPhase==6 || (lampPhase>9 && lampPhase<14)) lampOn = true;
    WOS_SetLampState(LAMP_4_TOP_LANE, (HatchlingsProgress&HATCHLING_ZONE_4) && lampOn);
    WOS_SetLampState(LAMP_5_TOP_LANE, (HatchlingsProgress&HATCHLING_ZONE_5) && lampOn);
    WOS_SetLampState(LAMP_6_TOP_LANE, (HatchlingsProgress&HATCHLING_ZONE_6) && lampOn);
    if (HatchlingsProgress&HATCHLING_LANE_TO_TOP) {
      WOS_SetLampState(LAMP_INLANE_50k, (HatchlingsProgress&HATCHLING_50K_RIGHT) && lampOn);
      WOS_SetLampState(LAMP_INLANE_EXTRA_BALL, (HatchlingsProgress&HATCHLING_EXTRA) && lampOn);
      WOS_SetLampState(LAMP_INLANE_SPECIAL, (HatchlingsProgress&HATCHLING_SPECIAL_RIGHT) && lampOn);
    } else {
      WOS_SetLampState(LAMP_INLANE_50k, LaneToTopValue[CurrentPlayer]==1);
      WOS_SetLampState(LAMP_INLANE_EXTRA_BALL, LaneToTopValue[CurrentPlayer]==2);
      WOS_SetLampState(LAMP_INLANE_SPECIAL, LaneToTopValue[CurrentPlayer]==3);
    }
  } else if (CurrentWeaponSelected==WEAPON_INDEX_AIR_ASSAULT) {
    byte lampPhase = ((CurrentTime-GameModeStartTime)/100)%24;
    if (lampPhase<12) {
      WOS_SetLampState(LAMP_4_TOP_LANE, !(lampPhase%3));
      WOS_SetLampState(LAMP_5_TOP_LANE, !(lampPhase%3));
      WOS_SetLampState(LAMP_6_TOP_LANE, !(lampPhase%3));
      WOS_SetLampState(LAMP_INLANE_50k, (lampPhase%4)==0);
      WOS_SetLampState(LAMP_INLANE_EXTRA_BALL, (lampPhase%4)==1);
      WOS_SetLampState(LAMP_INLANE_SPECIAL, (lampPhase%4)==2);
    } else {
      WOS_SetLampState(LAMP_4_TOP_LANE, (lampPhase%4)==0);
      WOS_SetLampState(LAMP_5_TOP_LANE, (lampPhase%4)==1);
      WOS_SetLampState(LAMP_6_TOP_LANE, (lampPhase%4)==2);
      WOS_SetLampState(LAMP_INLANE_50k, !(lampPhase%3));
      WOS_SetLampState(LAMP_INLANE_EXTRA_BALL, !(lampPhase%3));
      WOS_SetLampState(LAMP_INLANE_SPECIAL, !(lampPhase%3));
    }
  } else if (GameMode==GAME_MODE_PRE_PLUNGE_CHOICE) {
    WOS_SetLampState(LAMP_INLANE_50k, 0);
    WOS_SetLampState(LAMP_INLANE_EXTRA_BALL, 0);
    WOS_SetLampState(LAMP_INLANE_SPECIAL, 0);
    WOS_SetLampState(LAMP_4_TOP_LANE, SkillShotLane==0x01, 0, 210);
    WOS_SetLampState(LAMP_5_TOP_LANE, SkillShotLane==0x02, 0, 210);
    WOS_SetLampState(LAMP_6_TOP_LANE, SkillShotLane==0x04, 0, 210);
  } else if (GameMode==GAME_MODE_SKILL_SHOT) {
    WOS_SetLampState(LAMP_INLANE_50k, 0);
    WOS_SetLampState(LAMP_INLANE_EXTRA_BALL, 0);
    WOS_SetLampState(LAMP_INLANE_SPECIAL, 0);
    WOS_SetLampState(LAMP_4_TOP_LANE, SkillShotLane==0x01, 0, 210);
    WOS_SetLampState(LAMP_5_TOP_LANE, SkillShotLane==0x02, 0, 210);
    WOS_SetLampState(LAMP_6_TOP_LANE, SkillShotLane==0x04, 0, 210);
  } else if (GameMode==GAME_MODE_BATTLE_MULTIBALL || GameMode==GAME_MODE_DEFEND_MULTIBALL) {
    boolean engaged = (CurrentWeaponsEngaged[CurrentPlayer]&WEAPONS_MODE_AIR_ASSAULT)?true:false;
    if (engaged) {
      byte lampPhase = (CurrentTime/100)%3;
      WOS_SetLampState(LAMP_INLANE_50k, lampPhase==0);
      WOS_SetLampState(LAMP_INLANE_EXTRA_BALL, lampPhase==1);
      WOS_SetLampState(LAMP_INLANE_SPECIAL, lampPhase==2);
      WOS_SetLampState(LAMP_4_TOP_LANE, (TopLaneStatus[CurrentPlayer]&0x01), 0, 100);
      WOS_SetLampState(LAMP_5_TOP_LANE, (TopLaneStatus[CurrentPlayer]&0x02), 0, 100);
      WOS_SetLampState(LAMP_6_TOP_LANE, (TopLaneStatus[CurrentPlayer]&0x04), 0, 100);
    } else {
      WOS_SetLampState(LAMP_INLANE_50k, 0);
      WOS_SetLampState(LAMP_INLANE_EXTRA_BALL, 0);
      WOS_SetLampState(LAMP_INLANE_SPECIAL, 0);
      WOS_SetLampState(LAMP_4_TOP_LANE, 0);
      WOS_SetLampState(LAMP_5_TOP_LANE, 0);
      WOS_SetLampState(LAMP_6_TOP_LANE, 0);
    }
  } else {
    boolean engaged = (CurrentWeaponsEngaged[CurrentPlayer]&WEAPONS_MODE_AIR_ASSAULT)?true:false;
    boolean unfinished = (WeaponsStatus[CurrentPlayer][WEAPON_INDEX_AIR_ASSAULT]<100)?true:false;
    if (SkillShotLaneHitTime) {
      WOS_SetLampState(LAMP_4_TOP_LANE, SkillShotLaneHit==0, 0, 250);
      WOS_SetLampState(LAMP_5_TOP_LANE, SkillShotLaneHit==1, 0, 250);
      WOS_SetLampState(LAMP_6_TOP_LANE, SkillShotLaneHit==2, 0, 250);
      if (CurrentTime>(SkillShotLaneHitTime+3000)) SkillShotLaneHitTime = 0;
    } else if (TopLanesCompletedTime) {
      byte lampPhase = (CurrentTime/100)%4;
      WOS_SetLampState(LAMP_4_TOP_LANE, lampPhase, 0);
      WOS_SetLampState(LAMP_5_TOP_LANE, (lampPhase+1)%4, 0);
      WOS_SetLampState(LAMP_6_TOP_LANE, (lampPhase+2)%4, 0);
      if (CurrentTime>(TopLanesCompletedTime+5000)) {
        TopLanesCompletedTime = 0;
      }
    } else {
      WOS_SetLampState(LAMP_4_TOP_LANE, (TopLaneStatus[CurrentPlayer]&0x01) || engaged, 0, ((TopLaneStatus[CurrentPlayer]&0x01)==0 && unfinished)?300:0);
      WOS_SetLampState(LAMP_5_TOP_LANE, (TopLaneStatus[CurrentPlayer]&0x02) || engaged, 0, ((TopLaneStatus[CurrentPlayer]&0x02)==0 && unfinished)?300:0);
      WOS_SetLampState(LAMP_6_TOP_LANE, (TopLaneStatus[CurrentPlayer]&0x04) || engaged, 0, ((TopLaneStatus[CurrentPlayer]&0x04)==0 && unfinished)?300:0);
    }

    if (engaged) {
      byte lampPhase = (CurrentTime/150)%3;
      WOS_SetLampState(LAMP_INLANE_50k, lampPhase==0, 0);
      WOS_SetLampState(LAMP_INLANE_EXTRA_BALL, lampPhase==1, 0);
      WOS_SetLampState(LAMP_INLANE_SPECIAL, lampPhase==2, 0);
    } else {
      WOS_SetLampState(LAMP_INLANE_50k, LaneToTopValue[CurrentPlayer]==1);
      WOS_SetLampState(LAMP_INLANE_EXTRA_BALL, LaneToTopValue[CurrentPlayer]==2);
      WOS_SetLampState(LAMP_INLANE_SPECIAL, LaneToTopValue[CurrentPlayer]==3);
    }

  }
}


void ShowLeftSaucerLamps() {

  if (GameMode==GAME_MODE_CHASE_FINAL_SHOT) {
      byte lampPhase = ((CurrentTime-GameModeStartTime)/50)%16;
      for (int count=0;count<10;count++) {
        WOS_SetLampState(LAMP_LEFT_1K+count, (count==lampPhase)||(count==lampPhase-1)||(count==lampPhase-2), (count==lampPhase-2));
      }
      WOS_SetLampState(LAMP_LEFT_SAUCER, lampPhase<8, 0, 200);
  } else if (CurrentWeaponSelected==WEAPON_INDEX_BLUE_LASER) {
    byte lampPhase = ((CurrentTime-GameModeStartTime)/100)%16;
    for (int count=0;count<10;count++) {
      WOS_SetLampState(LAMP_LEFT_1K+count, (count==lampPhase)||(count==lampPhase-1)||(count==lampPhase-2), (count==lampPhase-2));
    }
    WOS_SetLampState(LAMP_LEFT_SAUCER, lampPhase<8, 0, 100);
  } else if (GameMode==GAME_MODE_SKILL_SHOT || GameMode==GAME_MODE_PRE_PLUNGE_CHOICE) {
    for (int count=0;count<10;count++) {
      WOS_SetLampState(LAMP_LEFT_1K+count, 0);
    }
    if ((CurrentWeaponSelected%2)) WOS_SetLampState(LAMP_LEFT_SAUCER, 1, 1, 50);
    else WOS_SetLampState(LAMP_LEFT_SAUCER, 0);
  } else if (GameMode==GAME_MODE_ADD_LEFT_WEAPON) {
    for (int count=0;count<10;count++) {
      WOS_SetLampState(LAMP_LEFT_1K+count, 0);
    }
    WOS_SetLampState(LAMP_LEFT_SAUCER, 1, 1, 50);
  } else if (GameMode==GAME_MODE_DEFEND_MULTIBALL && DefendLeftPowerUpStart!=0) {
    if (CurrentTime<(DefendLeftPowerUpStart+8000)) {
      byte lampPhase = 1 + (CurrentTime-DefendLeftPowerUpStart)/800;
      for (int count=0;count<10;count++) {
        WOS_SetLampState(LAMP_LEFT_1K+count, count<lampPhase);
      }
    } else if (CurrentTime<(DefendLeftPowerUpStart+8512)) {
      for (int count=0;count<10;count++) {
        WOS_SetLampState(LAMP_LEFT_1K+count, 0);
      }
    } else {
      byte lampPhase = (CurrentTime/200)%2;
      int lampFlash = (CurrentTime%10)*25 + 50;
      for (int count=0;count<10;count++) {
        WOS_SetLampState(LAMP_LEFT_1K+count, (count%2)==lampPhase, 0, lampFlash);
      }
    }
    WOS_SetLampState(LAMP_LEFT_SAUCER, 1);
  } else if (GameMode==GAME_MODE_DEFEND_MULTIBALL && DefendRightPowerUpStart!=0) {
    for (int count=0;count<10;count++) {
      WOS_SetLampState(LAMP_LEFT_1K+count, 0);
    }
    WOS_SetLampState(LAMP_LEFT_SAUCER, CurrentTime>(DefendRightPowerUpStart+8512));
  } else if (GameMode==GAME_MODE_DEFEND_MULTIBALL) {
    for (int count=0;count<10;count++) {
      WOS_SetLampState(LAMP_LEFT_1K+count, (CreatureDefense/10)>=count, 0, 125);
    }
    WOS_SetLampState(LAMP_LEFT_SAUCER, CreatureDefense>=100);
  } else if (GameMode==GAME_MODE_BATTLE_MULTIBALL) {
    if (CreatureHitsLeft<11) {
      for (int count=0;count<10;count++) {
        WOS_SetLampState(LAMP_LEFT_1K+count, (CreatureHitsLeft-1)>=count, 0, 125);
      }
      WOS_SetLampState(LAMP_LEFT_SAUCER, 0);
    } else {
      byte lampPhase = ((CurrentTime-GameModeStartTime)/100)%16;
      for (int count=0;count<10;count++) {
        WOS_SetLampState(LAMP_LEFT_1K+count, (count==lampPhase)||(count==lampPhase-1)||(count==lampPhase-2), (count==lampPhase-2));
      }
      WOS_SetLampState(LAMP_LEFT_SAUCER, lampPhase<8, 0, 100);
    }
  } else if (GameMode==GAME_MODE_VICTORY) {
    if (LeftVictoryReady) {
      byte lampPhase = ((CurrentTime-GameModeStartTime)/100)%16;
      for (int count=0;count<10;count++) {
        WOS_SetLampState(LAMP_LEFT_1K+count, (count==lampPhase)||(count==lampPhase-1)||(count==lampPhase-2), (count==lampPhase-2));
      }
      WOS_SetLampState(LAMP_LEFT_SAUCER, lampPhase<8, 0, 100);
    } else {
      for (int count=0;count<10;count++) {
        WOS_SetLampState(LAMP_LEFT_1K+count, 0);
      }
      WOS_SetLampState(LAMP_LEFT_SAUCER, 0);
    }
  } else if (GameMode==GAME_MODE_BATTLE_FINISHING_SHOT || GameMode==GAME_MODE_WAIT_TO_RESTART_BATTLE) {
    byte lampPhase = ((CurrentTime-GameModeStartTime)/100)%16;
    for (int count=0;count<10;count++) {
      WOS_SetLampState(LAMP_LEFT_1K+count, (count==lampPhase)||(count==lampPhase-1)||(count==lampPhase-2), (count==lampPhase-2));
    }
    WOS_SetLampState(LAMP_LEFT_SAUCER, lampPhase<8, 0, 100);
  } else {
    if (LeftWeaponChangedTime) {
      if (CurrentTime>(LeftWeaponChangedTime+3000)) {
        LeftWeaponChangedTime = 0;
      }
      byte maxStrength = GetMaximumWeaponStrength(false);
      for (int count=0;count<10;count++) {
        WOS_SetLampState(LAMP_LEFT_1K+count, (maxStrength/10)>count, 0, 125);
      }
    } else {
      if (BonusLeftChanged==0) {
        byte showBonus = BonusLeft[CurrentPlayer];
        boolean engaged = (CurrentWeaponsEngaged[CurrentPlayer]&WEAPONS_MODE_BLUE_LASER)?true:false;
        boolean unfinished = (WeaponsStatus[CurrentPlayer][WEAPON_INDEX_BLUE_LASER]<100)?true:false;
        if (!unfinished) engaged = false;
        byte engagedPhase = (CurrentTime/100)%9;
        
        WOS_SetLampState(LAMP_LEFT_10K, (showBonus>=10), 0, (showBonus>10)?750:0);
        if (showBonus>10) {
          showBonus -= 10;
        }
        for (int count=0;count<9;count++) {
          WOS_SetLampState(LAMP_LEFT_1K+count, count<showBonus || (engaged && count==engagedPhase), (engaged && count==engagedPhase)?2:0);
        }
      } else {
        byte rollupPhase = (CurrentTime-BonusLeftChanged)/30;
        WOS_SetLampState(LAMP_LEFT_10K, (rollupPhase>=10), 0, (rollupPhase>10)?750:0);
        if (rollupPhase>10) {
          rollupPhase -= 10;
        }
        for (int count=0;count<9;count++) WOS_SetLampState(LAMP_LEFT_1K+count, count<rollupPhase);
        if (rollupPhase>=BonusLeft[CurrentPlayer]) {
          BonusLeftChanged = 0;
        }
      }
    }
    if (GameMode>=GAME_MODE_CHASE_START && GameMode<=GAME_MODE_EGGS_LOST) {
      WOS_SetLampState(LAMP_LEFT_SAUCER, 0);
    } else {
      WOS_SetLampState(LAMP_LEFT_SAUCER, LockStatus[CurrentPlayer]&(LOCK_STATUS_BLUE_READY|LOCK_STATUS_BLUE_LOCKED), 0, LockStatus[CurrentPlayer]&(LOCK_STATUS_BLUE_LOCKED)?0:400);
    }
  }
}


void ShowRightSaucerLamps() {

  if (GameMode==GAME_MODE_CHASE_FINAL_SHOT) {
    byte lampPhase = ((CurrentTime-GameModeStartTime)/50)%16;
    for (int count=0;count<10;count++) {
      WOS_SetLampState(LAMP_RIGHT_1K+count, (count==lampPhase)||(count==lampPhase-1)||(count==lampPhase-2), (count==lampPhase-2));
    }
    WOS_SetLampState(LAMP_RIGHT_SAUCER, lampPhase<8, 0, 200);
  } else if (CurrentWeaponSelected==WEAPON_INDEX_GREEN_LASER) {
    byte lampPhase = ((CurrentTime-GameModeStartTime)/100)%16;
    for (int count=0;count<10;count++) {
      WOS_SetLampState(LAMP_RIGHT_1K+count, (count==lampPhase)||(count==lampPhase-1)||(count==lampPhase-2), (count==lampPhase-2));
    }
    WOS_SetLampState(LAMP_RIGHT_SAUCER, lampPhase<8, 0, 100);
  } else if (GameMode==GAME_MODE_SKILL_SHOT || GameMode==GAME_MODE_PRE_PLUNGE_CHOICE) {
    for (int count=0;count<10;count++) {
      WOS_SetLampState(LAMP_RIGHT_1K+count, 0);
    }
    if ((CurrentWeaponSelected%2)==0) WOS_SetLampState(LAMP_RIGHT_SAUCER, 1, 1, 50);
    else WOS_SetLampState(LAMP_RIGHT_SAUCER, 0);
  } else if (GameMode==GAME_MODE_ADD_RIGHT_WEAPON) {
    for (int count=0;count<10;count++) {
      WOS_SetLampState(LAMP_RIGHT_1K+count, 0);
    }
    WOS_SetLampState(LAMP_RIGHT_SAUCER, 1, 1, 50);
  } else if (GameMode==GAME_MODE_DEFEND_MULTIBALL && DefendRightPowerUpStart!=0) {
    if (CurrentTime<(DefendRightPowerUpStart+8000)) {
      byte lampPhase = 1 + (CurrentTime-DefendRightPowerUpStart)/800;
      for (int count=0;count<10;count++) {
        WOS_SetLampState(LAMP_RIGHT_1K+count, count<lampPhase);
      }
    } else if (CurrentTime<(DefendRightPowerUpStart+8512)) {
      for (int count=0;count<10;count++) {
        WOS_SetLampState(LAMP_RIGHT_1K+count, 0);
      }
    } else {
      byte lampPhase = (CurrentTime/200)%2;
      int lampFlash = (CurrentTime%10)*25 + 50;
      for (int count=0;count<10;count++) {
        WOS_SetLampState(LAMP_RIGHT_1K+count, (count%2)==lampPhase, 0, lampFlash);
      }
    }
    WOS_SetLampState(LAMP_RIGHT_SAUCER, 1);
  } else if (GameMode==GAME_MODE_DEFEND_MULTIBALL && DefendLeftPowerUpStart!=0) {
    for (int count=0;count<10;count++) {
      WOS_SetLampState(LAMP_RIGHT_1K+count, 0);
    }
    WOS_SetLampState(LAMP_RIGHT_SAUCER, CurrentTime>(DefendLeftPowerUpStart+8512));
  } else if (GameMode==GAME_MODE_DEFEND_MULTIBALL) {
    for (int count=0;count<10;count++) {
      WOS_SetLampState(LAMP_RIGHT_1K+count, (CreatureDefense/10)>=count, 0, 125);
    }
    WOS_SetLampState(LAMP_RIGHT_SAUCER, CreatureDefense>=100);
  } else if (GameMode==GAME_MODE_BATTLE_MULTIBALL) {
    if (CreatureHitsRight<11) {
      for (int count=0;count<10;count++) {
        WOS_SetLampState(LAMP_RIGHT_1K+count, (CreatureHitsRight-1)>=count, 0, 125);
      }
      WOS_SetLampState(LAMP_RIGHT_SAUCER, 0);
    } else {
      byte lampPhase = ((CurrentTime-GameModeStartTime)/100)%16;
      for (int count=0;count<10;count++) {
        WOS_SetLampState(LAMP_RIGHT_1K+count, (count==lampPhase)||(count==lampPhase-1)||(count==lampPhase-2), (count==lampPhase-2));
      }
      WOS_SetLampState(LAMP_RIGHT_SAUCER, lampPhase<8, 0, 100);
    }
  } else if (GameMode==GAME_MODE_VICTORY) {
    if (RightVictoryReady) {
      byte lampPhase = ((CurrentTime-GameModeStartTime)/100)%16;
      for (int count=0;count<10;count++) {
        WOS_SetLampState(LAMP_RIGHT_1K+count, (count==lampPhase)||(count==lampPhase-1)||(count==lampPhase-2), (count==lampPhase-2));
      }
      WOS_SetLampState(LAMP_RIGHT_SAUCER, lampPhase<8, 0, 100);
    } else {
      for (int count=0;count<10;count++) {
        WOS_SetLampState(LAMP_RIGHT_1K+count, 0);
      }
      WOS_SetLampState(LAMP_RIGHT_SAUCER, 0);
    }
  } else if (GameMode==GAME_MODE_BATTLE_FINISHING_SHOT || GameMode==GAME_MODE_WAIT_TO_RESTART_BATTLE) {
    byte lampPhase = ((CurrentTime-GameModeStartTime)/100)%16;
    for (int count=0;count<10;count++) {
      WOS_SetLampState(LAMP_RIGHT_1K+count, (count==lampPhase)||(count==lampPhase-1)||(count==lampPhase-2), (count==lampPhase-2));
    }
    WOS_SetLampState(LAMP_RIGHT_SAUCER, lampPhase<8, 0, 100);
  } else {
    if (RightWeaponChangedTime) {
      if (CurrentTime>(RightWeaponChangedTime+3000)) {
        RightWeaponChangedTime = 0;
      }
      byte maxStrength = GetMaximumWeaponStrength(true);
      for (int count=0;count<10;count++) {
        WOS_SetLampState(LAMP_RIGHT_1K+count, (maxStrength/10)>count, 0, 125);
      }
    } else {
      if (BonusRightChanged==0) {
        byte showBonus = BonusRight[CurrentPlayer];
        boolean engaged = (CurrentWeaponsEngaged[CurrentPlayer]&WEAPONS_MODE_GREEN_LASER)?true:false;
        boolean unfinished = (WeaponsStatus[CurrentPlayer][WEAPON_INDEX_GREEN_LASER]<100)?true:false;
        if (!unfinished) engaged = false;
        byte engagedPhase = (CurrentTime/100)%9;
        WOS_SetLampState(LAMP_RIGHT_10K, (showBonus>=10), 0, (showBonus>10)?750:0);
        if (showBonus>10) {
          showBonus -= 10;
        }
        for (int count=0;count<9;count++) WOS_SetLampState(LAMP_RIGHT_1K+count, count<showBonus || (engaged && count==engagedPhase), (engaged && count==engagedPhase)?2:0);
      } else {
        byte rollupPhase = (CurrentTime-BonusRightChanged)/30;
        WOS_SetLampState(LAMP_RIGHT_10K, (rollupPhase>=10), 0, (rollupPhase>10)?750:0);
        if (rollupPhase>10) {
          rollupPhase -= 10;
        }
        for (int count=0;count<9;count++) WOS_SetLampState(LAMP_RIGHT_1K+count, count<rollupPhase);
        if (rollupPhase>=BonusRight[CurrentPlayer]) {
          BonusRightChanged = 0;
        }
      }
    }
    if (GameMode>=GAME_MODE_CHASE_START && GameMode<=GAME_MODE_EGGS_LOST) {
      WOS_SetLampState(LAMP_RIGHT_SAUCER, 0);
    } else {
      WOS_SetLampState(LAMP_RIGHT_SAUCER, LockStatus[CurrentPlayer]&(LOCK_STATUS_GREEN_READY|LOCK_STATUS_GREEN_LOCKED), 0, LockStatus[CurrentPlayer]&(LOCK_STATUS_GREEN_LOCKED)?0:400);
    }
  }
  
}


void ShowBonusXLamps() {
  if (GameMode==GAME_MODE_SKILL_SHOT || GameMode==GAME_MODE_PRE_PLUNGE_CHOICE) {
    for (int count=0;count<3;count++) {
      WOS_SetLampState(LAMP_BONUS_2X+count, 0);
    }
  } else if (GameMode==GAME_MODE_BATTLE_MULTIBALL) {
    for (int count=0;count<3;count++) {
      WOS_SetLampState(LAMP_BONUS_2X+count, 0);
    }
  } else {
    WOS_SetLampState(LAMP_BONUS_2X, BonusX[CurrentPlayer]==2 || BonusX[CurrentPlayer]==4 || BonusX[CurrentPlayer]==7 || BonusX[CurrentPlayer]==10, 0, (BonusX[CurrentPlayer]==4)?250:0);
    WOS_SetLampState(LAMP_BONUS_3X, BonusX[CurrentPlayer]==3 || BonusX[CurrentPlayer]==6 || BonusX[CurrentPlayer]==8 || BonusX[CurrentPlayer]==10, 0, (BonusX[CurrentPlayer]==6)?250:0);
    WOS_SetLampState(LAMP_BONUS_5X, BonusX[CurrentPlayer]==5 || BonusX[CurrentPlayer]>6);
  }
}


void ShowStandupLamps() {

  if (StoriesEngaged[CurrentPlayer]==STORY_EGGS) {
      byte lampPhase = ((CurrentTime/100)%20);
      boolean lampOn = false;
      if (lampPhase==0 || lampPhase==3 || lampPhase==6 || (lampPhase>9 && lampPhase<14)) lampOn = true;
      WOS_SetLampState(LAMP_1_STANDUP, (HatchlingsProgress&HATCHLING_ZONE_1) && lampOn);
      WOS_SetLampState(LAMP_2_STANDUP, (HatchlingsProgress&HATCHLING_ZONE_2) && lampOn);
      WOS_SetLampState(LAMP_3_STANDUP, (HatchlingsProgress&HATCHLING_ZONE_3) && lampOn);
  } else if (CurrentWeaponSelected==WEAPON_INDEX_HARPOONS) {
    byte lampPhase = ((CurrentTime-GameModeStartTime)/100)%24;

    if (lampPhase<12) {
      for (int count=0;count<3;count++) {
        WOS_SetLampState(LAMP_1_STANDUP+count, !(lampPhase%3));
      }
    } else {
      for (int count=0;count<3;count++) {
        WOS_SetLampState(LAMP_1_STANDUP+count, (lampPhase%3)==count);
      }
    }
  } else if (GameMode==GAME_MODE_PRE_PLUNGE_CHOICE) {  
    for (int count=0;count<3;count++) {
      WOS_SetLampState(LAMP_1_STANDUP+count, 0);
    }
  } else if (GameMode==GAME_MODE_SKILL_SHOT) {
    if (SuperSkillShotReady) {
      WOS_SetLampState(LAMP_1_STANDUP, 1, 0, 100);
      WOS_SetLampState(LAMP_2_STANDUP, 1, 0, 100);
      WOS_SetLampState(LAMP_3_STANDUP, 1, 0, 100);
    }
  } else if (GameMode==GAME_MODE_BATTLE_MULTIBALL) {
    boolean engaged = (CurrentWeaponsEngaged[CurrentPlayer]&WEAPONS_MODE_HARPOONS)?true:false;
    if (engaged) {
      WOS_SetLampState(LAMP_1_STANDUP, (StandupStatus[CurrentPlayer]&0x01), 0, 100);
      WOS_SetLampState(LAMP_2_STANDUP, (StandupStatus[CurrentPlayer]&0x02), 0, 100);
      WOS_SetLampState(LAMP_3_STANDUP, (StandupStatus[CurrentPlayer]&0x04), 0, 100);
    } else {
      for (int count=0;count<3;count++) {
        WOS_SetLampState(LAMP_1_STANDUP+count, 0);
      }
    }
  } else {
    boolean engaged = (CurrentWeaponsEngaged[CurrentPlayer]&WEAPONS_MODE_HARPOONS)?true:false;
    boolean unfinished = (WeaponsStatus[CurrentPlayer][WEAPON_INDEX_HARPOONS]<100)?true:false;
    
    if (StandupsCompletedTime) {
      byte lampPhase = (CurrentTime/100)%3;
      WOS_SetLampState(LAMP_1_STANDUP, lampPhase==0);
      WOS_SetLampState(LAMP_2_STANDUP, lampPhase==1);
      WOS_SetLampState(LAMP_3_STANDUP, lampPhase==2);
      if (CurrentTime>(StandupsCompletedTime+5000)) {
        StandupsCompletedTime = 0;
      }
    } else {
      WOS_SetLampState(LAMP_1_STANDUP, (StandupStatus[CurrentPlayer]&0x01)||engaged, 0, (StandupStatus[CurrentPlayer]&0x01)==0 && unfinished?300:0);
      WOS_SetLampState(LAMP_2_STANDUP, (StandupStatus[CurrentPlayer]&0x02)||engaged, 0, (StandupStatus[CurrentPlayer]&0x02)==0 && unfinished?300:0);
      WOS_SetLampState(LAMP_3_STANDUP, (StandupStatus[CurrentPlayer]&0x04)||engaged, 0, (StandupStatus[CurrentPlayer]&0x04)==0 && unfinished?300:0);
    }
  }
}


void Show3DropLamps() {

  if (StoriesEngaged[CurrentPlayer]==STORY_EGGS && (HatchlingsProgress&HATCHLING_6DROP_BANK)) {
    byte lampPhase = ((CurrentTime/100)%20);
    boolean lampOn = false;
    if (lampPhase==0 || lampPhase==3 || lampPhase==6 || (lampPhase>9 && lampPhase<14)) lampOn = true;
    WOS_SetLampState(LAMP_3_DROPS_10K, lampOn && (HatchlingsProgress&HATCHLING_10K));
    WOS_SetLampState(LAMP_3_DROPS_20K, lampOn && (HatchlingsProgress&HATCHLING_20K));
    WOS_SetLampState(LAMP_3_DROPS_30K, lampOn && (HatchlingsProgress&HATCHLING_30K));
    WOS_SetLampState(LAMP_3_DROPS_50K, lampOn && (HatchlingsProgress&HATCHLING_50K));
    WOS_SetLampState(LAMP_3_DROPS_SPECIAL, lampOn && (HatchlingsProgress&HATCHLING_SPECIAL));  
  } else if (CurrentWeaponSelected==WEAPON_INDEX_DEPTH_CHARGES) {
    byte lampPhase = ((CurrentTime-GameModeStartTime)/100)%24;
    if (lampPhase<9) {
      for (int count=0;count<5;count++) {
        WOS_SetLampState(LAMP_3_DROPS_10K+count, !(lampPhase%3));
      }
    } else {
      for (int count=0;count<5;count++) {
        WOS_SetLampState(LAMP_3_DROPS_10K+count, ((lampPhase-9)%5)==count);
      }
    }
  } else if (GameMode==GAME_MODE_SKILL_SHOT || GameMode==GAME_MODE_PRE_PLUNGE_CHOICE) {
    for (int count=0;count<5;count++) {
      WOS_SetLampState(LAMP_3_DROPS_10K+count, 0);
    }
  } else if (GameMode==GAME_MODE_VICTORY) {
    for (int count=0;count<5;count++) {
      WOS_SetLampState(LAMP_3_DROPS_10K+count, 1, 0, LeftVictoryReady?0:200);
    }
  } else if (GameMode==GAME_MODE_BATTLE_MULTIBALL) {
    boolean engaged = (CurrentWeaponsEngaged[CurrentPlayer]&WEAPONS_MODE_DEPTH_CHARGES)?true:false;

    if (engaged) {
      byte lampPhase = ((CurrentTime-GameModeStartTime)/100)%40;
      if (lampPhase<20) {
        for (int count=0;count<5;count++) {
          WOS_SetLampState(LAMP_3_DROPS_10K+count, lampPhase%2);
        }
      } else {
        for (int count=0;count<5;count++) {
          WOS_SetLampState(LAMP_3_DROPS_10K+count, (lampPhase%5)==count);
        }
      }
    } else {
      for (int count=0;count<5;count++) {
        WOS_SetLampState(LAMP_3_DROPS_10K+count, 0);
      }
    }
    
  } else {
    boolean engaged = (CurrentWeaponsEngaged[CurrentPlayer]&WEAPONS_MODE_DEPTH_CHARGES)?true:false;
    boolean unfinished = (WeaponsStatus[CurrentPlayer][WEAPON_INDEX_DEPTH_CHARGES]<100)?true:false;
    
    for (int count=0;count<5;count++) {
//      WOS_SetLampState(LAMP_3_DROPS_10K+count, ((count)==Drop6Clears[CurrentPlayer]) || engaged, ((count)!=Drop6Clears[CurrentPlayer])?2:0, ((count)!=Drop6Clears[CurrentPlayer] && unfinished)?300:0);
      WOS_SetLampState(LAMP_3_DROPS_10K+count, ((count)==Drop6Clears[CurrentPlayer]) || engaged, 0, ((count)!=Drop6Clears[CurrentPlayer] && unfinished)?300:0);
    }
  }
}

void ShowTimerLamps() {
  WOS_SetLampState(LAMP_COLLECT_DOUBLE_TIMER, DoubleTimerBonus[CurrentPlayer]);
  WOS_SetLampState(LAMP_DOUBLE_TIMER_OUTLANES, DoubleTimerBonus[CurrentPlayer]);

  if (StoriesEngaged[CurrentPlayer]==STORY_EGGS && (HatchlingsProgress&HATCHLING_LITES_OUT)) {
    byte lampPhase = ((CurrentTime/100)%20);
    boolean lampOn = false;
    if (lampPhase==0 || lampPhase==3 || lampPhase==6 || (lampPhase>9 && lampPhase<14)) lampOn = true;
    WOS_SetLampState(LAMP_5_DROP_LIGHTS_OUTLANES, lampOn);
  } else if (CurrentWeaponSelected==WEAPON_INDEX_TORPEDOES) {
    byte lampPhase = ((CurrentTime-GameModeStartTime)/100)%8;
    WOS_SetLampState(LAMP_5_DROP_LIGHTS_OUTLANES, lampPhase==0 || lampPhase==3);
  } else if (GameMode>=GAME_MODE_CHASE_START && GameMode<=GAME_MODE_CHASE_LOST) {
    WOS_SetLampState(LAMP_5_DROP_LIGHTS_OUTLANES, 0);
  } else if (GameMode==GAME_MODE_VICTORY) {
    WOS_SetLampState(LAMP_5_DROP_LIGHTS_OUTLANES, 1, 0, RightVictoryReady?0:200);
  } else {
    boolean engaged = (CurrentWeaponsEngaged[CurrentPlayer]&WEAPONS_MODE_TORPEDOES)&&(((CurrentTime/200)%3)==0)?true:false;
    boolean unfinished = (WeaponsStatus[CurrentPlayer][WEAPON_INDEX_TORPEDOES]<100)?true:false;
    WOS_SetLampState(LAMP_5_DROP_LIGHTS_OUTLANES, engaged&&unfinished);
  }
}



void ShowPopBumperLamps() {

  if (StoriesEngaged[CurrentPlayer]==STORY_EGGS && (HatchlingsProgress&HATCHLING_POP_BUMPERS)) {
    byte lampPhase = ((CurrentTime/100)%20);
    boolean lampOn = false;
    if (lampPhase==0 || lampPhase==3 || lampPhase==6 || (lampPhase>9 && lampPhase<14)) lampOn = true;
    WOS_SetLampState(LAMP_SPINNER_AND_LOWER_POP, lampOn && (HatchlingsProgress&HATCHLING_SPINNER));
    WOS_SetLampState(LAMP_RIGHT_POP, lampOn && (HatchlingsProgress&HATCHLING_RIGHT_POP));
    WOS_SetLampState(LAMP_TOP_POP, lampOn && (HatchlingsProgress&HATCHLING_UPPER_POP));
  } else if (CurrentWeaponSelected==WEAPON_INDEX_CANNONS) {
    byte lampPhase = ((CurrentTime-GameModeStartTime)/100)%24;

    if (lampPhase<12) {
      for (int count=0;count<3;count++) {
        WOS_SetLampState(LAMP_RIGHT_POP+count, !(lampPhase%3));
      }
    } else {
      for (int count=0;count<3;count++) {
        WOS_SetLampState(LAMP_RIGHT_POP+count, (lampPhase%3)==count);
      }
    }
  } else if (GameMode==GAME_MODE_SKILL_SHOT || GameMode==GAME_MODE_PRE_PLUNGE_CHOICE) {
    for (int count=0;count<3;count++) {
      WOS_SetLampState(LAMP_RIGHT_POP+count, 0);
    }
  } else if (GameMode==GAME_MODE_BATTLE_MULTIBALL) {
    boolean engaged = (CurrentWeaponsEngaged[CurrentPlayer]&WEAPONS_MODE_CANNONS)?true:false;
    if (engaged) {
      for (int count=0;count<3;count++) {
        WOS_SetLampState(LAMP_RIGHT_POP+count, 1, 0, 100);
      }
    } else {
      for (int count=0;count<3;count++) {
        WOS_SetLampState(LAMP_RIGHT_POP+count, 0);
      }
    }
  } else if (GameMode==GAME_MODE_CHASE_SPINNER) {
    WOS_SetLampState(LAMP_SPINNER_AND_LOWER_POP, 1, 0, 125);
    WOS_SetLampState(LAMP_RIGHT_POP, 0);
    WOS_SetLampState(LAMP_TOP_POP, 0);
  } else {

    boolean rightRecent = (CurrentTime<(LastRightPopHit+500))?true:false;
    boolean upperRecent = (CurrentTime<(LastUpperPopHit+500))?true:false;
    boolean lowerRecent = (CurrentTime<(LastLowerPopHit+500))?true:false;
    
    if (lowerRecent || upperRecent || rightRecent) {
      WOS_SetLampState(LAMP_RIGHT_POP, rightRecent, 0, 25);
      WOS_SetLampState(LAMP_TOP_POP, upperRecent, 0, 25);
      WOS_SetLampState(LAMP_SPINNER_AND_LOWER_POP, lowerRecent, 0, 25);
    } else {
      boolean engaged = (CurrentWeaponsEngaged[CurrentPlayer]&WEAPONS_MODE_CANNONS)?true:false;
      boolean unfinished = (WeaponsStatus[CurrentPlayer][WEAPONS_MODE_CANNONS]<100)?true:false;

      if (engaged) {
        for (int count=0;count<3;count++) {
          WOS_SetLampState(LAMP_RIGHT_POP+count, engaged, 0, unfinished?300:0);
        }
      } else {
        boolean lowerHoldoverActive = TopLaneLevel[CurrentPlayer]>=1 && StandupLevel[CurrentPlayer]>=1;
        boolean rightHoldoverActive = TopLaneLevel[CurrentPlayer]>=2 && StandupLevel[CurrentPlayer]>=2;
        boolean topHoldoverActive = TopLaneLevel[CurrentPlayer]>=3 && StandupLevel[CurrentPlayer]>=3;
        WOS_SetLampState(LAMP_RIGHT_POP, rightHoldoverActive, 0, (HoldoverAwards[CurrentPlayer]&HOLDOVER_BONUS)?0:500);
        WOS_SetLampState(LAMP_TOP_POP, topHoldoverActive, 0, (HoldoverAwards[CurrentPlayer]&HOLDOVER_BONUS_X)?0:500);
        WOS_SetLampState(LAMP_SPINNER_AND_LOWER_POP, lowerHoldoverActive, 0, (HoldoverAwards[CurrentPlayer]&HOLDOVER_MULTIBALL_TIMER)?0:500);
      }
    }
  }

}


void ShowShootAgainLamps() {

  if ( (BallFirstSwitchHitTime==0 && BallSaveNumSeconds) || (BallSaveEndTime && CurrentTime<BallSaveEndTime) ) {
    unsigned long msRemaining = 5000;
    if (BallSaveEndTime!=0) msRemaining = BallSaveEndTime - CurrentTime;
    WOS_SetLampState(LAMP_SHOOT_AGAIN, 1, 0, (msRemaining < 5000) ? 100 : 500);
    WOS_SetLampState(LAMP_HEAD_SAME_PLAYER_SHOOTS_AGAIN, 1, 0, (msRemaining < 5000) ? 100 : 500);
  } else {
    WOS_SetLampState(LAMP_SHOOT_AGAIN, SamePlayerShootsAgain);
    WOS_SetLampState(LAMP_HEAD_SAME_PLAYER_SHOOTS_AGAIN, SamePlayerShootsAgain);
  }
}


byte ScannerAnimationLength[LAMP_ANIMATION_STEPS] = {
  34, 33, 32, 31, 30, 29, 28, 27, 26, 26, 25, 25, 24, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14
};

byte UnchangedLamps[NUM_LAMP_ANIMATION_BYTES] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x81, 0xFF, 0xFF
};

byte ScanBlips[7][NUM_LAMP_ANIMATION_BYTES] = {
  {0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00}, 
  {0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00}, 
  {0x00, 0x00, 0x00, 0xE0, 0x00, 0x00, 0x00, 0x00}, 
  {0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0x00}, 
  {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
};
byte ScanStartFrame[7] = {19, 23, 20, 21, 18, 19, 15};
//byte ScanStartFrame[7] = {11, 15, 12, 13, 10, 11, 7};

void SetChaseScannerBlips(byte blipSet, byte lampState) {
  // Turn on/off all the scanner lamps
  byte lampNum = 0;

  byte *blips = ScanBlips[blipSet];
  byte curBitmask;  
  for (byte count=0; count<NUM_LAMP_ANIMATION_BYTES; count++) {
    if (*blips) { 
      curBitmask = 0x01;
      for (byte bitCount=0; bitCount<8; bitCount++) {
        if ((*blips)&curBitmask) WOS_SetLampState(lampNum, lampState, 0, 25);
        curBitmask *= 2;
        lampNum += 1;
      }
    } else {
      lampNum += 8;
    }
    blips += 1;
  }  
}


boolean IsScannedLampOn(unsigned long frameStartTime, unsigned long curTime, byte frameStartNum, byte curFrame) {
  unsigned long onTime;
  if (curFrame==LAMP_ANIMATION_STEPS) {
    curFrame = LAMP_ANIMATION_STEPS + (curTime-frameStartTime)/14;
    frameStartTime = frameStartTime + curFrame*14;
    onTime = 4;
  } else {
    onTime = ScannerAnimationLength[frameStartNum] / 3;
  }

  if ((curTime-frameStartTime)<=onTime) return true;
  return true;
}

byte LastLampPhaseShown = 0xFF;

void ShowChaseScannerLamps(unsigned long animationTime, byte scannedLampMode) {

  if (scannedLampMode<GAME_MODE_CHASE_SPINNER || scannedLampMode>GAME_MODE_CHASE_5_DROPS) return;

  byte curUnchangedLamps[NUM_LAMP_ANIMATION_BYTES];
  
  for (byte count=0; count<NUM_LAMP_ANIMATION_BYTES; count++) {
    curUnchangedLamps[count] = UnchangedLamps[count] | ScanBlips[scannedLampMode-GAME_MODE_CHASE_SPINNER][count];
  }

  byte lampPhase = 0;
  unsigned long frameAnimationStart = 0;
  for (lampPhase=0; lampPhase<LAMP_ANIMATION_STEPS; lampPhase++) {
    frameAnimationStart += ScannerAnimationLength[lampPhase];
    if (frameAnimationStart>animationTime) {
      frameAnimationStart -= ScannerAnimationLength[lampPhase];
      break;
    }
  }

  if (lampPhase!=LastLampPhaseShown) {
    LastLampPhaseShown = lampPhase;
  
    byte scanStartFrame = ScanStartFrame[scannedLampMode-GAME_MODE_CHASE_SPINNER];
  
    if (lampPhase==0) {
      // blank out blips if we're at frame 0
      SetChaseScannerBlips(scannedLampMode-GAME_MODE_CHASE_SPINNER, 0);
    } else if (lampPhase>=scanStartFrame) {
      // turn on flashing blips
      SetChaseScannerBlips(scannedLampMode-GAME_MODE_CHASE_SPINNER, 1);
    }
  
  
    byte lampNum = 0;
  
    if (lampPhase<LAMP_ANIMATION_STEPS) {
      ShowLampAnimationSingleStep(1, lampPhase, curUnchangedLamps);
    } else if (lampPhase==LAMP_ANIMATION_STEPS) {
  
      // Turn off all lamps except the unchanged ones
      byte *lampsPtr = curUnchangedLamps;
      lampNum = 0;
      byte curBitmask;
      
      for (byte count=0; count<NUM_LAMP_ANIMATION_BYTES; count++) {
        byte lampsByte = ~(*lampsPtr);
        
        curBitmask = 0x01;
        if (lampsByte) {
          for (byte bitCount=0; bitCount<8; bitCount++) {
            if (lampsByte&curBitmask) WOS_SetLampState(count*8 + bitCount, 0);
            curBitmask *= 2;
          }
        } else {
          lampNum += 8;
        }
        lampsPtr += 1;
      }
    }
  }
  
}


boolean RequestedGIState;
unsigned long GIOverrideEndTime;

void SetGeneralIlluminationOn(boolean generalIlluminationOn = true) {
  RequestedGIState = generalIlluminationOn;
  if (GIOverrideEndTime) return;  
  WOS_SetContinuousSolenoid(!generalIlluminationOn, SOL_GI_RELAY);
}

void OverrideGeneralIllumination(boolean generalIlluminationOn, unsigned long endTime) {
  GIOverrideEndTime = endTime;  
  WOS_SetContinuousSolenoid(!generalIlluminationOn, SOL_GI_RELAY);
}


////////////////////////////////////////////////////////////////////////////
//
//  Display Management functions
//
////////////////////////////////////////////////////////////////////////////
unsigned long LastTimeScoreChanged = 0;
unsigned long LastFlashOrDash = 0;
unsigned long ScoreOverrideValue[4] = {0, 0, 0, 0};
byte LastAnimationSeed[4] = {0, 0, 0, 0};
byte AnimationStartSeed[4] = {0, 0, 0, 0};
byte ScoreOverrideStatus = 0;
byte ScoreAnimation[4] = {0, 0, 0, 0};
byte AnimationDisplayOrder[4] = {0, 1, 2, 3};
#define DISPLAY_OVERRIDE_BLANK_SCORE 0xFFFFFFFF
#define DISPLAY_OVERRIDE_ANIMATION_NONE     0
#define DISPLAY_OVERRIDE_ANIMATION_BOUNCE   1
#define DISPLAY_OVERRIDE_ANIMATION_FLUTTER  2
#define DISPLAY_OVERRIDE_ANIMATION_FLYBY    3
#define DISPLAY_OVERRIDE_ANIMATION_CENTER   4
byte LastScrollPhase = 0;

byte MagnitudeOfScore(unsigned long score) {
  if (score == 0) return 0;

  byte retval = 0;
  while (score > 0) {
    score = score / 10;
    retval += 1;
  }
  return retval;
}


void OverrideScoreDisplay(byte displayNum, unsigned long value, byte animationType) {
  if (displayNum > 3) return;

  ScoreOverrideStatus |= (0x01 << displayNum);
  ScoreAnimation[displayNum] = animationType;
  ScoreOverrideValue[displayNum] = value;
  LastAnimationSeed[displayNum] = 255;
}

byte GetDisplayMask(byte numDigits) {
  byte displayMask = 0;
  for (byte digitCount = 0;digitCount < numDigits;digitCount++) {
#ifdef WOS_USE_7_DIGIT_DISPLAYS
    displayMask |= (0x40 >> digitCount);
#else
    displayMask |= (0x20 >> digitCount);
#endif
  }
  return displayMask;
}


void SetAnimationDisplayOrder(byte disp0, byte disp1, byte disp2, byte disp3) {
  AnimationDisplayOrder[0] = disp0;
  AnimationDisplayOrder[1] = disp1;
  AnimationDisplayOrder[2] = disp2;
  AnimationDisplayOrder[3] = disp3;
}


void ShowAnimatedValue(byte displayNum, unsigned long displayScore, byte animationType) {
  byte overrideAnimationSeed;
  byte displayMask = WOS_ALL_DIGITS_MASK;

  byte numDigits = MagnitudeOfScore(displayScore);
  if (numDigits == 0) numDigits = 1;
  if (numDigits < (WOS_NUM_DIGITS - 1) && animationType == DISPLAY_OVERRIDE_ANIMATION_BOUNCE) {
    // This score is going to be animated (back and forth)
    overrideAnimationSeed = (CurrentTime / 250) % (2 * WOS_NUM_DIGITS - 2 * numDigits);
    if (overrideAnimationSeed != LastAnimationSeed[displayNum]) {

      LastAnimationSeed[displayNum] = overrideAnimationSeed;
      byte shiftDigits = (overrideAnimationSeed);
      if (shiftDigits >= ((WOS_NUM_DIGITS + 1) - numDigits)) shiftDigits = (WOS_NUM_DIGITS - numDigits) * 2 - shiftDigits;
      byte digitCount;
      displayMask = GetDisplayMask(numDigits);
      for (digitCount = 0;digitCount < shiftDigits;digitCount++) {
        displayScore *= 10;
        displayMask = displayMask >> 1;
      }
      //WOS_SetDisplayBlank(displayNum, 0x00);
      WOS_SetDisplay(displayNum, displayScore, false);
      WOS_SetDisplayBlank(displayNum, displayMask);
    }
  } else if (animationType == DISPLAY_OVERRIDE_ANIMATION_FLUTTER) {
    overrideAnimationSeed = CurrentTime / 50;
    if (overrideAnimationSeed != LastAnimationSeed[displayNum]) {
      LastAnimationSeed[displayNum] = overrideAnimationSeed;
      displayMask = GetDisplayMask(numDigits);
      if (overrideAnimationSeed % 2) {
        displayMask &= 0x55;
      } else {
        displayMask &= 0xAA;
      }
      WOS_SetDisplay(displayNum, displayScore, false);
      WOS_SetDisplayBlank(displayNum, displayMask);
    }
  } else if (animationType == DISPLAY_OVERRIDE_ANIMATION_FLYBY) {
    overrideAnimationSeed = (CurrentTime / 75) % 256;
    if (overrideAnimationSeed != LastAnimationSeed[displayNum]) {
      if (LastAnimationSeed[displayNum] == 255) {
        AnimationStartSeed[displayNum] = overrideAnimationSeed;
      }
      LastAnimationSeed[displayNum] = overrideAnimationSeed;

      byte realAnimationSeed = overrideAnimationSeed - AnimationStartSeed[displayNum];
      if (overrideAnimationSeed < AnimationStartSeed[displayNum]) realAnimationSeed = (255 - AnimationStartSeed[displayNum]) + overrideAnimationSeed;

      if (realAnimationSeed > 34) {
        WOS_SetDisplayBlank(displayNum, 0x00);
        ScoreOverrideStatus &= ~(0x01 << displayNum);
      } else {
        int shiftDigits = (-6 * ((int)AnimationDisplayOrder[displayNum] + 1)) + realAnimationSeed;
        displayMask = GetDisplayMask(numDigits);
        if (shiftDigits < 0) {
          shiftDigits = 0 - shiftDigits;
          byte digitCount;
          for (digitCount = 0;digitCount < shiftDigits;digitCount++) {
            displayScore /= 10;
            displayMask = displayMask << 1;
          }
        } else if (shiftDigits > 0) {
          byte digitCount;
          for (digitCount = 0;digitCount < shiftDigits;digitCount++) {
            displayScore *= 10;
            displayMask = displayMask >> 1;
          }
        }
        WOS_SetDisplay(displayNum, displayScore, false);
        WOS_SetDisplayBlank(displayNum, displayMask);
      }
    }
  } else if (animationType == DISPLAY_OVERRIDE_ANIMATION_CENTER) {
    overrideAnimationSeed = CurrentTime / 250;
    if (overrideAnimationSeed != LastAnimationSeed[displayNum]) {
      LastAnimationSeed[displayNum] = overrideAnimationSeed;
      byte shiftDigits = (WOS_NUM_DIGITS - numDigits) / 2;

      byte digitCount;
      displayMask = GetDisplayMask(numDigits);
      for (digitCount = 0;digitCount < shiftDigits;digitCount++) {
        displayScore *= 10;
        displayMask = displayMask >> 1;
      }
      //WOS_SetDisplayBlank(displayNum, 0x00);
      WOS_SetDisplay(displayNum, displayScore, false);
      WOS_SetDisplayBlank(displayNum, displayMask);
    }
  } else {
    WOS_SetDisplay(displayNum, displayScore, true, 1);
  }

}

void ShowPlayerScores(byte displayToUpdate, boolean flashCurrent, boolean dashCurrent, unsigned long allScoresShowValue = 0) {

  if (displayToUpdate == 0xFF) ScoreOverrideStatus = 0;
  byte displayMask = WOS_ALL_DIGITS_MASK;
  unsigned long displayScore = 0;
  byte scrollPhaseChanged = false;

  byte scrollPhase = ((CurrentTime - LastTimeScoreChanged) / 125) % 16;
  if (scrollPhase != LastScrollPhase) {
    LastScrollPhase = scrollPhase;
    scrollPhaseChanged = true;
  }

  for (byte scoreCount = 0; scoreCount < 4; scoreCount++) {

    // If this display is currently being overriden, then we should update it
    if (allScoresShowValue == 0 && (ScoreOverrideStatus & (0x01 << scoreCount))) {
      displayScore = ScoreOverrideValue[scoreCount];
      if (displayScore != DISPLAY_OVERRIDE_BLANK_SCORE) {
        ShowAnimatedValue(scoreCount, displayScore, ScoreAnimation[scoreCount]);
      } else {
        WOS_SetDisplayBlank(scoreCount, 0);
      }

    } else {
      boolean showingCurrentAchievement = false;
      // No override, update scores designated by displayToUpdate
      if (allScoresShowValue == 0) {
        displayScore = CurrentScores[scoreCount];
        displayScore += (CurrentAchievements[scoreCount] % 10);
        if (CurrentAchievements[scoreCount]) showingCurrentAchievement = true;
      }
      else displayScore = allScoresShowValue;

      // If we're updating all displays, or the one currently matching the loop, or if we have to scroll
      if (displayToUpdate == 0xFF || displayToUpdate == scoreCount || displayScore > WOS_MAX_DISPLAY_SCORE || showingCurrentAchievement) {

        // Don't show this score if it's not a current player score (even if it's scrollable)
        if (displayToUpdate == 0xFF && (scoreCount >= CurrentNumPlayers && CurrentNumPlayers != 0) && allScoresShowValue == 0) {
          WOS_SetDisplayBlank(scoreCount, 0x00);
          continue;
        }

        if (displayScore > WOS_MAX_DISPLAY_SCORE) {
          // Score needs to be scrolled
          if ((CurrentTime - LastTimeScoreChanged) < 2000) {
            // show score for four seconds after change
            WOS_SetDisplay(scoreCount, displayScore % (WOS_MAX_DISPLAY_SCORE + 1), false);
            byte blank = WOS_ALL_DIGITS_MASK;
            if (showingCurrentAchievement && (CurrentTime / 200) % 2) {
              blank &= ~(0x01 << (WOS_NUM_DIGITS - 1));
            }
            WOS_SetDisplayBlank(scoreCount, blank);
          } else {
            // Scores are scrolled 10 digits and then we wait for 6
            if (scrollPhase < 11 && scrollPhaseChanged) {
              byte numDigits = MagnitudeOfScore(displayScore);

              // Figure out top part of score
              unsigned long tempScore = displayScore;
              if (scrollPhase < WOS_NUM_DIGITS) {
                displayMask = WOS_ALL_DIGITS_MASK;
                for (byte scrollCount = 0;scrollCount < scrollPhase;scrollCount++) {
                  displayScore = (displayScore % (WOS_MAX_DISPLAY_SCORE + 1)) * 10;
                  displayMask = displayMask >> 1;
                }
              } else {
                displayScore = 0;
                displayMask = 0x00;
              }

              // Add in lower part of score
              if ((numDigits + scrollPhase) > 10) {
                byte numDigitsNeeded = (numDigits + scrollPhase) - 10;
                for (byte scrollCount = 0;scrollCount < (numDigits - numDigitsNeeded);scrollCount++) {
                  tempScore /= 10;
                }
                displayMask |= GetDisplayMask(MagnitudeOfScore(tempScore));
                displayScore += tempScore;
              }
              WOS_SetDisplayBlank(scoreCount, displayMask);
              WOS_SetDisplay(scoreCount, displayScore);
            }
          }
        } else {
          if (flashCurrent && displayToUpdate == scoreCount) {
            unsigned long flashSeed = CurrentTime / 250;
            if (flashSeed != LastFlashOrDash) {
              LastFlashOrDash = flashSeed;
              if (((CurrentTime / 250) % 2) == 0) WOS_SetDisplayBlank(scoreCount, 0x00);
              else WOS_SetDisplay(scoreCount, displayScore, true, 2);
            }
          } else if (dashCurrent && displayToUpdate == scoreCount) {
            unsigned long dashSeed = CurrentTime / 50;
            if (dashSeed != LastFlashOrDash) {
              LastFlashOrDash = dashSeed;
              byte dashPhase = (CurrentTime / 60) % (2 * WOS_NUM_DIGITS * 3);
              byte numDigits = MagnitudeOfScore(displayScore);
              if (dashPhase < (2 * WOS_NUM_DIGITS)) {
                displayMask = GetDisplayMask((numDigits == 0) ? 2 : numDigits);
                if (dashPhase < (WOS_NUM_DIGITS + 1)) {
                  for (byte maskCount = 0;maskCount < dashPhase;maskCount++) {
                    displayMask &= ~(0x01 << maskCount);
                  }
                } else {
                  for (byte maskCount = (2 * WOS_NUM_DIGITS);maskCount > dashPhase;maskCount--) {
                    byte firstDigit = (0x20) << (WOS_NUM_DIGITS - 6);
                    displayMask &= ~(firstDigit >> (maskCount - dashPhase - 1));
                  }
                }
                WOS_SetDisplay(scoreCount, displayScore);
                WOS_SetDisplayBlank(scoreCount, displayMask);
              } else {
                WOS_SetDisplay(scoreCount, displayScore, true, 2);
              }
            }
          } else {
            byte blank = 0;
            blank = WOS_SetDisplay(scoreCount, displayScore, false, 2);
            if (showingCurrentAchievement && (CurrentTime / 200) % 2) {
              blank &= ~(0x01 << (WOS_NUM_DIGITS - 1));
            }
            WOS_SetDisplayBlank(scoreCount, blank);
          }
        }
      } // End if this display should be updated
    } // End on non-overridden
  } // End loop on scores

}

void ShowFlybyValue(byte numToShow, unsigned long timeBase) {
  byte shiftDigits = (CurrentTime - timeBase) / 120;
  byte rightSideBlank = 0;

  unsigned long bigVersionOfNum = (unsigned long)numToShow;
  for (byte count = 0;count < shiftDigits;count++) {
    bigVersionOfNum *= 10;
    rightSideBlank /= 2;
    if (count > 2) rightSideBlank |= 0x20;
  }
  bigVersionOfNum /= 1000;

  byte curMask = WOS_SetDisplay(CurrentPlayer, bigVersionOfNum, false, 0);
  if (bigVersionOfNum == 0) curMask = 0;
  WOS_SetDisplayBlank(CurrentPlayer, ~(~curMask | rightSideBlank));
}

void StartScoreAnimation(unsigned long scoreToAnimate) {
  if (ScoreAdditionAnimation != 0) {
    //CurrentScores[CurrentPlayer] += ScoreAdditionAnimation;
  }
  ScoreAdditionAnimation += scoreToAnimate;
  ScoreAdditionAnimationStartTime = CurrentTime;
  LastRemainingAnimatedScoreShown = 0;
}


////////////////////////////////////////////////////////////////////////////
//
//  Machine State Helper functions
//
////////////////////////////////////////////////////////////////////////////
boolean AddPlayer(boolean resetNumPlayers = false) {

  if (Credits < 1 && !FreePlayMode) return false;
  if (resetNumPlayers) CurrentNumPlayers = 0;
  if (CurrentNumPlayers >= 4) return false;

  CurrentNumPlayers += 1;
  WOS_SetDisplay(CurrentNumPlayers - 1, 0);
  WOS_SetDisplayBlank(CurrentNumPlayers - 1, 0x30);

  WOS_SetLampState(LAMP_HEAD_1_PLAYER, CurrentNumPlayers==1, 0, 500);
  WOS_SetLampState(LAMP_HEAD_2_PLAYERS, CurrentNumPlayers==2, 0, 500);
  WOS_SetLampState(LAMP_HEAD_3_PLAYERS, CurrentNumPlayers==3, 0, 500);
  WOS_SetLampState(LAMP_HEAD_4_PLAYERS, CurrentNumPlayers==4, 0, 500);

  if (!FreePlayMode) {
    Credits -= 1;
    WOS_WriteByteToEEProm(WOS_CREDITS_EEPROM_BYTE, Credits);
    WOS_SetDisplayCredits(Credits, !FreePlayMode);
    WOS_SetCoinLockout(false);
  }
  if (CurrentNumPlayers==1) Audio.StopAllAudio();
  else QueueNotification(SOUND_EFFECT_VP_ADD_PLAYER_1 + (CurrentNumPlayers - 1), 10);

  WOS_WriteULToEEProm(WOS_TOTAL_PLAYS_EEPROM_START_BYTE, WOS_ReadULFromEEProm(WOS_TOTAL_PLAYS_EEPROM_START_BYTE) + 1);

  return true;
}


unsigned short ChuteAuditByte[] = {WOS_CHUTE_1_COINS_START_BYTE, WOS_CHUTE_2_COINS_START_BYTE, WOS_CHUTE_3_COINS_START_BYTE};
void AddCoinToAudit(byte chuteNum) {
  if (chuteNum>2) return;
  unsigned short coinAuditStartByte = ChuteAuditByte[chuteNum];
  WOS_WriteULToEEProm(coinAuditStartByte, WOS_ReadULFromEEProm(coinAuditStartByte) + 1);
}


void AddCredit(boolean playSound = false, byte numToAdd = 1) {
  if (Credits < MaximumCredits) {
    Credits += numToAdd;
    if (Credits > MaximumCredits) Credits = MaximumCredits;
    WOS_WriteByteToEEProm(WOS_CREDITS_EEPROM_BYTE, Credits);
    if (playSound) {
      //PlaySoundEffect(SOUND_EFFECT_ADD_CREDIT);
      WOS_PushToSolenoidStack(SOL_KNOCKER, 8, true);// Williams of this era use the knocker for credits
    }
    WOS_SetDisplayCredits(Credits, !FreePlayMode);
    WOS_SetCoinLockout(false);
  } else {
    WOS_SetDisplayCredits(Credits, !FreePlayMode);
    WOS_SetCoinLockout(true);
  }

}

byte SwitchToChuteNum(byte switchHit) {
  byte chuteNum = 0;
  if (switchHit==SW_COIN_2) chuteNum = 1;
  else if (switchHit==SW_COIN_3) chuteNum = 2;
  return chuteNum;
}

boolean AddCoin(byte chuteNum) {
  boolean creditAdded = false;
  if (chuteNum>2) return false;
  byte cpcSelection = GetCPCSelection(chuteNum);

  // Find the lowest chute num with the same ratio selection
  // and use that ChuteCoinsInProgress counter
  byte chuteNumToUse;
  for (chuteNumToUse=0;chuteNumToUse<=chuteNum;chuteNumToUse++) {
    if (GetCPCSelection(chuteNumToUse)==cpcSelection) break;
  }

  PlaySoundEffect(SOUND_EFFECT_COIN_DROP_1+(CurrentTime%3));

  byte cpcCoins = GetCPCCoins(cpcSelection);
  byte cpcCredits = GetCPCCredits(cpcSelection);
  byte coinProgressBefore = ChuteCoinsInProgress[chuteNumToUse];
  ChuteCoinsInProgress[chuteNumToUse] += 1;

  if (ChuteCoinsInProgress[chuteNumToUse]==cpcCoins) {
    if (cpcCredits>cpcCoins) AddCredit(cpcCredits - (coinProgressBefore));
    else AddCredit(cpcCredits);
    ChuteCoinsInProgress[chuteNumToUse] = 0;
    creditAdded = true;
  } else {
    if (cpcCredits>cpcCoins) {
      AddCredit(1);
      creditAdded = true;
    } else {
    }
  }

  return creditAdded;
}


void AddSpecialCredit() {
  AddCredit(false, 1);
  WOS_PushToTimedSolenoidStack(SOL_KNOCKER, 8, CurrentTime, true);
  WOS_WriteULToEEProm(WOS_TOTAL_REPLAYS_EEPROM_START_BYTE, WOS_ReadULFromEEProm(WOS_TOTAL_REPLAYS_EEPROM_START_BYTE) + 1);
}

void AwardSpecial() {
  if (SpecialCollected) return;
  SpecialCollected = true;
  if (TournamentScoring) {
    CurrentScores[CurrentPlayer] += SpecialValue * TotalScoreMultiplier;
  } else {
    AddSpecialCredit();
  }
}

boolean AwardExtraBall() {
  if (ExtraBallCollected) return false;
  ExtraBallCollected = true;
  if (TournamentScoring) {
    CurrentScores[CurrentPlayer] += ExtraBallValue * TotalScoreMultiplier;
  } else {
    SamePlayerShootsAgain = true;
    WOS_SetLampState(LAMP_SHOOT_AGAIN, SamePlayerShootsAgain);
    WOS_SetLampState(LAMP_HEAD_SAME_PLAYER_SHOOTS_AGAIN, SamePlayerShootsAgain);
    QueueNotification(SOUND_EFFECT_VP_EXTRA_BALL, 8);
  }
  return true;
}


void IncreasePlayfieldMultiplier(unsigned long duration) {
  if (PlayfieldMultiplierExpiration) PlayfieldMultiplierExpiration += duration;
  else PlayfieldMultiplierExpiration = CurrentTime + duration;
  PlayfieldMultiplier += 1;
  if (PlayfieldMultiplier > 5) {
    PlayfieldMultiplier = 5;
  } else {
    QueueNotification(SOUND_EFFECT_VP_RETURN_TO_1X + (PlayfieldMultiplier - 1), 1);
  }
}


#define ADJ_TYPE_LIST                 1
#define ADJ_TYPE_MIN_MAX              2
#define ADJ_TYPE_MIN_MAX_DEFAULT      3
#define ADJ_TYPE_SCORE                4
#define ADJ_TYPE_SCORE_WITH_DEFAULT   5
#define ADJ_TYPE_SCORE_NO_DEFAULT     6
byte AdjustmentType = 0;
byte NumAdjustmentValues = 0;
byte AdjustmentValues[8];
byte CurrentAdjustmentStorageByte = 0;
byte TempValue = 0;
byte *CurrentAdjustmentByte = NULL;
unsigned long *CurrentAdjustmentUL = NULL;
unsigned long SoundSettingTimeout = 0;
unsigned long AdjustmentScore;



int RunSelfTest(int curState, boolean curStateChanged) {
  int returnState = curState;
  CurrentNumPlayers = 0;

  if (curStateChanged) {
    // Send a stop-all command and reset the sample-rate offset, in case we have
    //  reset while the WAV Trigger was already playing.
    Audio.StopAllAudio();
    WOS_TurnOffAllLamps();
//    PlaySoundEffect(SOUND_EFFECT_SELF_TEST_MODE_START-curState, 0, true);
    Audio.StopAllAudio();
//    int modeMapping = SelfTestStateToCalloutMap[-1 - curState];
//    Audio.PlaySound((unsigned short)modeMapping, AUDIO_PLAY_TYPE_WAV_TRIGGER, 10);
    Audio.PlaySound((unsigned short)(SOUND_EFFECT_SELF_TEST_MODE_START-curState), AUDIO_PLAY_TYPE_WAV_TRIGGER, 10);
  } else {
    if (SoundSettingTimeout && CurrentTime>SoundSettingTimeout) {
      SoundSettingTimeout = 0;
      Audio.StopAllAudio();
    }
  }

  // Any state that's greater than MACHINE_STATE_TEST_DONE is handled by the Base Self-test code
  // Any that's less, is machine specific, so we handle it here.
  if (curState >= MACHINE_STATE_TEST_DONE) {
    byte cpcSelection = 0xFF;
    byte chuteNum = 0xFF;
    if (curState==MACHINE_STATE_ADJUST_CPC_CHUTE_1) chuteNum = 0;
    if (curState==MACHINE_STATE_ADJUST_CPC_CHUTE_2) chuteNum = 1;
    if (curState==MACHINE_STATE_ADJUST_CPC_CHUTE_3) chuteNum = 2;
    if (chuteNum!=0xFF) cpcSelection = GetCPCSelection(chuteNum);
    returnState = RunBaseSelfTest(returnState, curStateChanged, CurrentTime, SW_CREDIT_RESET, SW_SLAM);
    if (chuteNum!=0xFF) {
      if (cpcSelection != GetCPCSelection(chuteNum)) {
        byte newCPC = GetCPCSelection(chuteNum);
        Audio.StopAllAudio();
        Audio.PlaySound(SOUND_EFFECT_SELF_TEST_CPC_START+newCPC, AUDIO_PLAY_TYPE_WAV_TRIGGER, 10);
      }
    }
  } else {
    byte curSwitch = WOS_PullFirstFromSwitchStack();

    if (curSwitch == SW_SELF_TEST_SWITCH && (CurrentTime - GetLastSelfTestChangedTime()) > 250) {
      SetLastSelfTestChangedTime(CurrentTime);
      if (WOS_GetUpDownSwitchState()) returnState -= 1;
      else returnState += 1;
    }

    if (curSwitch == SW_SLAM) {
      returnState = MACHINE_STATE_ATTRACT;
    }

    if (curStateChanged) {
      for (int count = 0;count < 4;count++) {
        WOS_SetDisplay(count, 0);
        WOS_SetDisplayBlank(count, 0x00);
      }
      WOS_SetDisplayCredits(MACHINE_STATE_TEST_BOOT - curState);
      WOS_SetDisplayBallInPlay(0, false);
      CurrentAdjustmentByte = NULL;
      CurrentAdjustmentUL = NULL;
      CurrentAdjustmentStorageByte = 0;

      AdjustmentType = ADJ_TYPE_MIN_MAX;
      AdjustmentValues[0] = 0;
      AdjustmentValues[1] = 1;
      TempValue = 0;

      switch (curState) {
        case MACHINE_STATE_ADJUST_FREEPLAY:
          CurrentAdjustmentByte = (byte *)&FreePlayMode;
          CurrentAdjustmentStorageByte = EEPROM_FREE_PLAY_BYTE;
          break;
        case MACHINE_STATE_ADJUST_BALL_SAVE:
          AdjustmentType = ADJ_TYPE_LIST;
          NumAdjustmentValues = 5;
          AdjustmentValues[1] = 5;
          AdjustmentValues[2] = 10;
          AdjustmentValues[3] = 15;
          AdjustmentValues[4] = 20;
          CurrentAdjustmentByte = &BallSaveNumSeconds;
          CurrentAdjustmentStorageByte = EEPROM_BALL_SAVE_BYTE;
          break;
        case MACHINE_STATE_ADJUST_SOUND_SELECTOR:
          AdjustmentType = ADJ_TYPE_MIN_MAX;
          AdjustmentValues[1] = 9;
          CurrentAdjustmentByte = &SoundSelector;
          CurrentAdjustmentStorageByte = EEPROM_SOUND_SELECTOR_BYTE;
          break;
        case MACHINE_STATE_ADJUST_MUSIC_VOLUME:
          AdjustmentType = ADJ_TYPE_MIN_MAX;
          AdjustmentValues[0] = 0;
          AdjustmentValues[1] = 10;
          CurrentAdjustmentByte = &MusicVolume;
          CurrentAdjustmentStorageByte = EEPROM_MUSIC_VOLUME_BYTE;
          break;
        case MACHINE_STATE_ADJUST_SFX_VOLUME:
          AdjustmentType = ADJ_TYPE_MIN_MAX;
          AdjustmentValues[0] = 0;
          AdjustmentValues[1] = 10;
          CurrentAdjustmentByte = &SoundEffectsVolume;
          CurrentAdjustmentStorageByte = EEPROM_SFX_VOLUME_BYTE;
          break;
        case MACHINE_STATE_ADJUST_CALLOUTS_VOLUME:
          AdjustmentType = ADJ_TYPE_MIN_MAX;
          AdjustmentValues[0] = 0;
          AdjustmentValues[1] = 10;
          CurrentAdjustmentByte = &CalloutsVolume;
          CurrentAdjustmentStorageByte = EEPROM_CALLOUTS_VOLUME_BYTE;
          break;
        case MACHINE_STATE_ADJUST_TOURNAMENT_SCORING:
          CurrentAdjustmentByte = (byte *)&TournamentScoring;
          CurrentAdjustmentStorageByte = EEPROM_TOURNAMENT_SCORING_BYTE;
          break;
        case MACHINE_STATE_ADJUST_TILT_WARNING:
          AdjustmentValues[1] = 2;
          CurrentAdjustmentByte = &MaxTiltWarnings;
          CurrentAdjustmentStorageByte = EEPROM_TILT_WARNING_BYTE;
          break;
        case MACHINE_STATE_ADJUST_AWARD_OVERRIDE:
          AdjustmentType = ADJ_TYPE_MIN_MAX_DEFAULT;
          AdjustmentValues[1] = 7;
          CurrentAdjustmentByte = &ScoreAwardReplay;
          CurrentAdjustmentStorageByte = EEPROM_AWARD_OVERRIDE_BYTE;
          break;
        case MACHINE_STATE_ADJUST_BALLS_OVERRIDE:
          AdjustmentType = ADJ_TYPE_LIST;
          NumAdjustmentValues = 3;
          AdjustmentValues[0] = 3;
          AdjustmentValues[1] = 5;
          AdjustmentValues[2] = 99;
          CurrentAdjustmentByte = &BallsPerGame;
          CurrentAdjustmentStorageByte = EEPROM_BALLS_OVERRIDE_BYTE;
          break;
        case MACHINE_STATE_ADJUST_SCROLLING_SCORES:
          CurrentAdjustmentByte = (byte *)&ScrollingScores;
          CurrentAdjustmentStorageByte = EEPROM_SCROLLING_SCORES_BYTE;
          break;
        case MACHINE_STATE_ADJUST_EXTRA_BALL_AWARD:
          AdjustmentType = ADJ_TYPE_SCORE_WITH_DEFAULT;
          CurrentAdjustmentUL = &ExtraBallValue;
          CurrentAdjustmentStorageByte = EEPROM_EXTRA_BALL_SCORE_UL;
          break;
        case MACHINE_STATE_ADJUST_SPECIAL_AWARD:
          AdjustmentType = ADJ_TYPE_SCORE_WITH_DEFAULT;
          CurrentAdjustmentUL = &SpecialValue;
          CurrentAdjustmentStorageByte = EEPROM_SPECIAL_SCORE_UL;
          break;
        case MACHINE_STATE_ADJUST_CREDIT_RESET_HOLD_TIME:
          AdjustmentType = ADJ_TYPE_LIST;
          NumAdjustmentValues = 5;
          AdjustmentValues[0] = 0;
          AdjustmentValues[1] = 1;
          AdjustmentValues[2] = 2;
          AdjustmentValues[3] = 3;
          AdjustmentValues[4] = 99;
          CurrentAdjustmentByte = &TimeRequiredToResetGame;
          CurrentAdjustmentStorageByte = EEPROM_CRB_HOLD_TIME; 
          break;
        case MACHINE_STATE_ADJUST_DONE:
          returnState = MACHINE_STATE_ATTRACT;
          break;
      }
    }

    // Change value, if the switch is hit
    if (curSwitch == SW_CREDIT_RESET) {

      if (CurrentAdjustmentByte && (AdjustmentType == ADJ_TYPE_MIN_MAX || AdjustmentType == ADJ_TYPE_MIN_MAX_DEFAULT)) {
        byte curVal = *CurrentAdjustmentByte;

        if (WOS_GetUpDownSwitchState()) {
          curVal += 1;
          if (curVal > AdjustmentValues[1]) {
            if (AdjustmentType == ADJ_TYPE_MIN_MAX) curVal = AdjustmentValues[0];
            else {
              if (curVal > 99) curVal = AdjustmentValues[0];
              else curVal = 99;
            }
          }
        } else {
          if (curVal==AdjustmentValues[0]) {            
            if (AdjustmentType==ADJ_TYPE_MIN_MAX_DEFAULT) curVal = 99;
            else curVal = AdjustmentValues[1];
          } else {
            curVal -= 1;
          }
        }

        *CurrentAdjustmentByte = curVal;
        if (CurrentAdjustmentStorageByte) EEPROM.write(CurrentAdjustmentStorageByte, curVal);

        if (curState==MACHINE_STATE_ADJUST_SOUND_SELECTOR) {
          Audio.StopAllAudio();
          Audio.PlaySound(SOUND_EFFECT_SELF_TEST_AUDIO_OPTIONS_START+curVal, AUDIO_PLAY_TYPE_WAV_TRIGGER, 10);
        } else if (curState==MACHINE_STATE_ADJUST_MUSIC_VOLUME) {
          if (SoundSettingTimeout) Audio.StopAllAudio();
          Audio.PlaySound(SOUND_EFFECT_BACKGROUND_SONG_1, AUDIO_PLAY_TYPE_WAV_TRIGGER, curVal);
          Audio.SetMusicVolume(curVal);
          SoundSettingTimeout = CurrentTime+5000;
        } else if (curState==MACHINE_STATE_ADJUST_SFX_VOLUME) {
          if (SoundSettingTimeout) Audio.StopAllAudio();
          Audio.PlaySound(SOUND_EFFECT_SW_LETTER_AWARDED, AUDIO_PLAY_TYPE_WAV_TRIGGER, curVal);
          Audio.SetSoundFXVolume(curVal);
          SoundSettingTimeout = CurrentTime+5000;
        } else if (curState==MACHINE_STATE_ADJUST_CALLOUTS_VOLUME) {
          if (SoundSettingTimeout) Audio.StopAllAudio();
          Audio.PlaySound(SOUND_EFFECT_VP_CANNONS, AUDIO_PLAY_TYPE_WAV_TRIGGER, curVal);
          Audio.SetNotificationsVolume(curVal);
          SoundSettingTimeout = CurrentTime+3000;
        } 
        
      } else if (CurrentAdjustmentByte && AdjustmentType == ADJ_TYPE_LIST) {
        byte valCount = 0;
        byte curVal = *CurrentAdjustmentByte;
        byte newIndex = 0;
        boolean upDownState = WOS_GetUpDownSwitchState();
        for (valCount = 0;valCount < (NumAdjustmentValues);valCount++) {
          if (curVal == AdjustmentValues[valCount]) {
            if (upDownState) {
              if (valCount<(NumAdjustmentValues-1)) newIndex = valCount + 1;
            } else {
              if (valCount>0) newIndex = valCount - 1;
            }
          }
        }
        *CurrentAdjustmentByte = AdjustmentValues[newIndex];
        if (CurrentAdjustmentStorageByte) EEPROM.write(CurrentAdjustmentStorageByte, AdjustmentValues[newIndex]);

        if (curState == MACHINE_STATE_ADJUST_CREDIT_RESET_HOLD_TIME) {
          Audio.StopAllAudio();
          if (*CurrentAdjustmentByte!=99) PlaySoundEffect(SOUND_EFFECT_AP_CRB_OPTION_1 + *CurrentAdjustmentByte);
          else PlaySoundEffect(SOUND_EFFECT_AP_CRB_OPTION_99);
        }        
      } else if (CurrentAdjustmentUL && (AdjustmentType == ADJ_TYPE_SCORE_WITH_DEFAULT || AdjustmentType == ADJ_TYPE_SCORE_NO_DEFAULT)) {
        unsigned long curVal = *CurrentAdjustmentUL;
        if (WOS_GetUpDownSwitchState()) curVal += 5000;
        else if (curVal>=5000) curVal -= 5000;
        if (curVal > 100000) curVal = 0;
        if (AdjustmentType == ADJ_TYPE_SCORE_NO_DEFAULT && curVal == 0) curVal = 5000;
        *CurrentAdjustmentUL = curVal;
        if (CurrentAdjustmentStorageByte) WOS_WriteULToEEProm(CurrentAdjustmentStorageByte, curVal);
      }

    }

    // Show current value
    if (CurrentAdjustmentByte != NULL) {
      WOS_SetDisplay(0, (unsigned long)(*CurrentAdjustmentByte), true);
    } else if (CurrentAdjustmentUL != NULL) {
      WOS_SetDisplay(0, (*CurrentAdjustmentUL), true);
    }

  }

  if (returnState == MACHINE_STATE_ATTRACT) {
    // If any variables have been set to non-override (99), return
    // them to dip switch settings
    // Balls Per Game, Player Loses On Ties, Novelty Scoring, Award Score
    //    DecodeDIPSwitchParameters();
    WOS_SetDisplayCredits(Credits, !FreePlayMode);
    ReadStoredParameters();
  }

  return returnState;
}




////////////////////////////////////////////////////////////////////////////
//
//  Audio Output functions
//
////////////////////////////////////////////////////////////////////////////
void PlayBackgroundSong(unsigned int songNum) {

  if (MusicVolume==0) return;

  Audio.PlayBackgroundSong(songNum);
}


unsigned long NextSoundEffectTime = 0;

void PlaySoundEffect(unsigned int soundEffectNum) {

  if (MachineState==MACHINE_STATE_INIT_GAMEPLAY) return;
  Audio.PlaySound(soundEffectNum, AUDIO_PLAY_TYPE_WAV_TRIGGER);

#if defined(WILLIAMS_TYPE_1_SOUND)
  // Only use Type 1 sound if we're below option 6
  if (SoundSelector<6) {
    switch (soundEffectNum) {
//      case SOUND_EFFECT_ADD_CREDIT:
//        WOS_PushToSolenoidStack(SOL_KNOCKER, 8, true);
//        break;
      case SOUND_EFFECT_EGGS_HATCHING:
        Audio.PlaySoundCardWhenPossible(16 * 256, CurrentTime, 0, 950, 8);
        Audio.PlaySoundCardWhenPossible(15 * 256, CurrentTime, 1000, 15, 8);
        break;
      case SOUND_EFFECT_BONUS_TIMER_PAYOUT:
        Audio.PlaySoundCardWhenPossible(15 * 256, CurrentTime, 0, 15, 5);
        break;
      case SOUND_EFFECT_BONUS_TIMER_EXPLOSION:
        Audio.PlaySoundCardWhenPossible(7 * 256, CurrentTime, 0, 15, 10);
        break;
      case SOUND_EFFECT_SMALL_EXPLOSION:
        Audio.PlaySoundCardWhenPossible(7 * 256, CurrentTime, 0, 15, 5);
        break;
      case SOUND_EFFECT_SLING_SHOT:
        Audio.PlaySoundCardWhenPossible(7 * 256, CurrentTime, 0, 55, 5);
        Audio.PlaySoundCardWhenPossible(19 * 256, CurrentTime, 60, 15, 5);
        break;
      case SOUND_EFFECT_SCORE_TICK:
        Audio.PlaySoundCardWhenPossible(7 * 256, CurrentTime, 0, 75, 5);
        Audio.PlaySoundCardWhenPossible(19 * 256, CurrentTime, 80, 15, 5);
        break;
      case SOUND_EFFECT_SCANNER_TICK:
        Audio.PlaySoundCardWhenPossible(22 * 256, CurrentTime, 0, 90, 7);
        Audio.PlaySoundCardWhenPossible(19 * 256, CurrentTime, 100, 15, 7);
        break;
      case SOUND_EFFECT_TILT_WARNING:
        Audio.PlaySoundCardWhenPossible(6 * 256, CurrentTime, 0, 1900, 100);
        Audio.PlaySoundCardWhenPossible(19 * 256, CurrentTime, 2000, 15, 100);
        break;
      case SOUND_EFFECT_TILT:
        Audio.PlaySoundCardWhenPossible(6 * 256, CurrentTime, 0, 1900, 100);
        Audio.PlaySoundCardWhenPossible(8 * 256, CurrentTime, 2000, 500, 100);
        break;
      case SOUND_EFFECT_OUTLANE_UNLIT:
/*        Audio.PlaySoundCardWhenPossible(30 * 256, CurrentTime, 0, 150, 90);
        Audio.PlaySoundCardWhenPossible(29 * 256, CurrentTime, 175, 150, 90);
        Audio.PlaySoundCardWhenPossible(28 * 256, CurrentTime, 350, 150, 90);
        Audio.PlaySoundCardWhenPossible(27 * 256, CurrentTime, 525, 150, 90);
        Audio.PlaySoundCardWhenPossible(26 * 256, CurrentTime, 700, 150, 90);
        Audio.PlaySoundCardWhenPossible(25 * 256, CurrentTime, 875, 150, 90);
*/
        Audio.PlaySoundCardWhenPossible(7 * 256, CurrentTime, 0, 150, 90);
        Audio.PlaySoundCardWhenPossible(24 * 256, CurrentTime, 175, 150, 90);
/*        Audio.PlaySoundCardWhenPossible(7 * 256, CurrentTime, 350, 150, 90);
        Audio.PlaySoundCardWhenPossible(8 * 256, CurrentTime, 525, 150, 90);
        Audio.PlaySoundCardWhenPossible(7 * 256, CurrentTime, 700, 150, 90);
        Audio.PlaySoundCardWhenPossible(24 * 256, CurrentTime, 875, 150, 90);
*/       
        Audio.PlaySoundCardWhenPossible(22 * 256, CurrentTime, 1050, 150, 90);
        break;
      case SOUND_EFFECT_OUTLANE_LIT:
        Audio.PlaySoundCardWhenPossible(21 * 256, CurrentTime, 0, 150, 90);
        Audio.PlaySoundCardWhenPossible(2 * 256, CurrentTime, 175, 150, 90);
        break;
      case SOUND_EFFECT_WEAPONS_SELECT_START:
        Audio.PlaySoundCardWhenPossible(16 * 256, CurrentTime, 0, 500, 90);
        Audio.PlaySoundCardWhenPossible(12 * 256, CurrentTime, 1500, 500, 90);
        Audio.PlaySoundCardWhenPossible(12 * 256, CurrentTime, 2500, 500, 90);
        Audio.PlaySoundCardWhenPossible(12 * 256, CurrentTime, 3500, 500, 90);
        break;
      case SOUND_EFFECT_BUMPER_HIT:
        Audio.PlaySoundCardWhenPossible(5 * 256, CurrentTime);
        break;
      case SOUND_EFFECT_FRENZY_BUMPER_HIT:
        Audio.PlaySoundCardWhenPossible(10 * 256, CurrentTime);
        break;
      case SOUND_EFFECT_LOCK_BOUNCED:
        Audio.PlaySoundCardWhenPossible(21 * 256, CurrentTime, 0, 500, 70);
        Audio.PlaySoundCardWhenPossible(23 * 256, CurrentTime, 500, 200, 75);
        break;
      case SOUND_EFFECT_STANDUP_DUPLICATE:
        Audio.PlaySoundCardWhenPossible(13 * 256, CurrentTime);
        break;
      case SOUND_EFFECT_STANDUP_HIT:
        Audio.PlaySoundCardWhenPossible(15 * 256, CurrentTime);
        break;
      case SOUND_EFFECT_LOWER_BUMPER_HIT:
        Audio.PlaySoundCardWhenPossible(10 * 256, CurrentTime);
        break;
//      case SOUND_EFFECT_10PT_SWITCH:
//        Audio.PlaySoundCardWhenPossible(3 * 256, CurrentTime);
//        break;
      case SOUND_EFFECT_BULLSEYE_LIT:
        Audio.PlaySoundCardWhenPossible(10 * 256, CurrentTime);
        break;
      case SOUND_EFFECT_BULLSEYE_UNLIT:
      case SOUND_EFFECT_CAPTIVE_BALL_UNLIT:
        Audio.PlaySoundCardWhenPossible(3 * 256, CurrentTime);
        break;
      case SOUND_EFFECT_MATCH_SPIN:
      case SOUND_EFFECT_SPINNER:
        Audio.PlaySoundCardWhenPossible(10 * 256, CurrentTime, 0, 50, 8);
        break;
      case SOUND_EFFECT_SPINNER_ALT:
        Audio.PlaySoundCardWhenPossible(20 * 256, CurrentTime, 0, 50, 8);
        break;
      case SOUND_EFFECT_BONUS_COUNT:
//        Audio.PlaySoundCardWhenPossible(20 * 256, CurrentTime, 0, 50, 90);
        Audio.PlaySoundCardWhenPossible(22 * 256, CurrentTime, 0, 50, 90);
        break;
      case SOUND_EFFECT_BONUS_COLLECT:
        Audio.PlaySoundCardWhenPossible(7 * 256, CurrentTime, 0, 75, 90);
        break;
      case SOUND_EFFECT_LEFT_WEAPON_FINISHED:
      case SOUND_EFFECT_RIGHT_WEAPON_FINISHED:
        Audio.PlaySoundCardWhenPossible(20 * 256, CurrentTime, 0, 50, 90);
        break;
      case SOUND_EFFECT_FLASH_FINISHED:
        Audio.PlaySoundCardWhenPossible(7 * 256, CurrentTime, 0, 50, 90);
        Audio.PlaySoundCardWhenPossible(7 * 256, CurrentTime, 400, 50, 91);
        Audio.PlaySoundCardWhenPossible(7 * 256, CurrentTime, 600, 50, 92);
        Audio.PlaySoundCardWhenPossible(8 * 256, CurrentTime, 900, 300, 93);
        break;
      case SOUND_EFFECT_MACHINE_START:
        if (SoundSelector==1) {
          Audio.PlaySoundCardWhenPossible(17 * 256, CurrentTime, 0, 20, 100);
          Audio.PlaySoundCardWhenPossible(25 * 256, CurrentTime, 50, 150, 100);
          Audio.PlaySoundCardWhenPossible(30 * 256, CurrentTime, 250, 150, 100);
          Audio.PlaySoundCardWhenPossible(17 * 256, CurrentTime, 500, 150, 100);
          Audio.PlaySoundCardWhenPossible(27 * 256, CurrentTime, 750, 150, 100);
          Audio.PlaySoundCardWhenPossible(30 * 256, CurrentTime, 1000, 150, 100);
          Audio.PlaySoundCardWhenPossible(29 * 256, CurrentTime, 1125, 150, 100);
          Audio.PlaySoundCardWhenPossible(18 * 256, CurrentTime, 1250, 150, 100);
          Audio.PlaySoundCardWhenPossible(17 * 256, CurrentTime, 1750, 150, 100);
          Audio.PlaySoundCardWhenPossible(18 * 256, CurrentTime, 2125, 150, 100);
          Audio.PlaySoundCardWhenPossible(19 * 256, CurrentTime, 2350, 20, 100);
          Audio.PlaySoundCardWhenPossible(22 * 256, CurrentTime, 2375, 150, 100);
        }
        break;
      case SOUND_EFFECT_TOP_LANE_REPEAT:
        Audio.PlaySoundCardWhenPossible(13 * 256, CurrentTime, 0, 50, 8);
        break;
      case SOUND_EFFECT_TOP_LANE_NEW:
        Audio.PlaySoundCardWhenPossible(15 * 256, CurrentTime, 0, 50, 8);
        break;
      case SOUND_EFFECT_SKILL_SHOT:
        Audio.PlaySoundCardWhenPossible(20 * 256, CurrentTime, 0, 10, 100);
        Audio.PlaySoundCardWhenPossible(22 * 256, CurrentTime, 250, 50, 8);
        break;
      case SOUND_EFFECT_STANDUPS_COMPLETE:
      case SOUND_EFFECT_TOP_LANE_LEVEL_FINISHED:
        Audio.PlaySoundCardWhenPossible(7 * 256, CurrentTime, 0, 100, 8);
        Audio.PlaySoundCardWhenPossible(7 * 256, CurrentTime, 100, 100, 8);
        Audio.PlaySoundCardWhenPossible(7 * 256, CurrentTime, 200, 100, 8);
        Audio.PlaySoundCardWhenPossible(9 * 256, CurrentTime, 300, 100, 8);
        break;
      case SOUND_EFFECT_SW_LETTER_AWARDED:
        Audio.PlaySoundCardWhenPossible(2 * 256, CurrentTime, 0, 50, 75);
        break;
      case SOUND_EFFECT_DROP_TARGET_HIT:
        Audio.PlaySoundCardWhenPossible(10 * 256, CurrentTime, 0, 250, 40);
        Audio.PlaySoundCardWhenPossible(8 * 256, CurrentTime, 260, 200, 45);
        Audio.PlaySoundCardWhenPossible(19 * 256, CurrentTime, 470, 200, 50);
        break;
      case SOUND_EFFECT_DROP_TARGET_RESET:
        Audio.PlaySoundCardWhenPossible(7 * 256, CurrentTime, 750, 50, 50);
        break;
      case SOUND_EFFECT_BATTLE_ALARM:
        Audio.PlaySoundCardWhenPossible(11 * 256, CurrentTime, 0, 2000, 90);
        Audio.PlaySoundCardWhenPossible(19 * 256, CurrentTime, 2100, 100, 95);
        break;
      case SOUND_EFFECT_BATTLE_LOST:
        Audio.PlaySoundCardWhenPossible(8 * 256, CurrentTime, 0, 2000, 90);
        break;
      case SOUND_EFFECT_BATTLE_WON:
        // Need an integrated light and sound show
        Audio.PlaySoundCardWhenPossible(24 * 256, CurrentTime, 1000, 3000, 90);
        Audio.PlaySoundCardWhenPossible(7 * 256, CurrentTime, 3100, 3300, 90);
        break;
//      case SOUND_EFFECT_WAITING_FOR_SKILL:
        /*
              Audio.PlaySoundCardWhenPossible(17*256, 0, 500, 10);
              Audio.PlaySoundCardWhenPossible(18*256, 500, 190, 10);
              Audio.PlaySoundCardWhenPossible(19*256, 700, 10, 100);
        */
        /*
              Audio.PlaySoundCardWhenPossible(28*256, 0, 100, 10);
              Audio.PlaySoundCardWhenPossible(28*256, 800, 100, 10);
              Audio.PlaySoundCardWhenPossible(28*256, 1600, 100, 10);
              Audio.PlaySoundCardWhenPossible(28*256, 2400, 100, 10);
              Audio.PlaySoundCardWhenPossible(29*256, 2800, 100, 10);
              Audio.PlaySoundCardWhenPossible(30*256, 3000, 100, 10);
              Audio.PlaySoundCardWhenPossible(28*256, 3200, 100, 10);
              Audio.PlaySoundCardWhenPossible(28*256, 4000, 100, 10);
              Audio.PlaySoundCardWhenPossible(28*256, 4800, 100, 10);
              Audio.PlaySoundCardWhenPossible(28*256, 5600, 100, 10);
              Audio.PlaySoundCardWhenPossible(29*256, 6000, 100, 10);
              Audio.PlaySoundCardWhenPossible(30*256, 6200, 100, 10);
        */
//        break;
      case SOUND_EFFECT_ENEMY_HIT:
        Audio.PlaySoundCardWhenPossible(7 * 256, CurrentTime, 0, 190, 90);
        break;
      case SOUND_EFFECT_ENEMY_RETALIATE:
        Audio.PlaySoundCardWhenPossible(13 * 256, CurrentTime, 0, 50, 90);
        break;
      case SOUND_EFFECT_ENEMY_CRITICAL_HIT:
        Audio.PlaySoundCardWhenPossible(7 * 256, CurrentTime, 0, 190, 90);
        Audio.PlaySoundCardWhenPossible(7 * 256, CurrentTime, 200, 190, 90);
        Audio.PlaySoundCardWhenPossible(7 * 256, CurrentTime, 400, 190, 90);
        Audio.PlaySoundCardWhenPossible(22 * 256, CurrentTime, 600, 2000, 90);
        break;
      case SOUND_EFFECT_AIR_ASSAULT:
        Audio.PlaySoundCardWhenPossible(22 * 256, CurrentTime, 500, 450, 90);
        Audio.PlaySoundCardWhenPossible(22 * 256, CurrentTime, 1000, 450, 90);
        Audio.PlaySoundCardWhenPossible(22 * 256, CurrentTime, 1500, 450, 90);
        Audio.PlaySoundCardWhenPossible(22 * 256, CurrentTime, 2000, 450, 90);
        Audio.PlaySoundCardWhenPossible(22 * 256, CurrentTime, 2500, 450, 90);
        Audio.PlaySoundCardWhenPossible(22 * 256, CurrentTime, 3000, 450, 90);
        break;
      case SOUND_EFFECT_ENEMY_SNEAK_ATTACK:
        Audio.PlaySoundCardWhenPossible(21 * 256, CurrentTime, 0, 900, 90);
        Audio.PlaySoundCardWhenPossible(19 * 256, CurrentTime, 1000, 50, 10);
        break;
      case SOUND_EFFECT_WIZARD_START_SAUCER:
        Audio.PlaySoundCardWhenPossible(22 * 256, CurrentTime, 0, 50, 90);
        break;
      case SOUND_EFFECT_BATTLE_MULTIBALL_START:
        Audio.PlaySoundCardWhenPossible(21 * 256, CurrentTime, 240, 250, 90);
        Audio.PlaySoundCardWhenPossible(11 * 256, CurrentTime, 1000, 900, 90);
        Audio.PlaySoundCardWhenPossible(8 * 256, CurrentTime, 1950, 10, 90);
        break;
    }
  }
#endif

}


void QueueNotification(unsigned int soundEffectNum, byte priority) {
  if (CalloutsVolume==0) return;
  if (SoundSelector<3 || SoundSelector==4 || SoundSelector==7 || SoundSelector==9) return;
  if (soundEffectNum < SOUND_EFFECT_VP_VOICE_NOTIFICATIONS_START || soundEffectNum >= (SOUND_EFFECT_VP_VOICE_NOTIFICATIONS_START + NUM_VOICE_NOTIFICATIONS)) return;

  Audio.QueuePrioritizedNotification(soundEffectNum, /*VoicePromptLengths[soundEffectNum-SOUND_EFFECT_VP_VOICE_NOTIFICATIONS_START]*/ 0, priority, CurrentTime);
}


void AlertPlayerUp(byte playerNum) {
  QueueNotification(SOUND_EFFECT_VP_PLAYER, 1);
  QueueNotification(SOUND_EFFECT_VP_ONE + playerNum, 1);
  QueueNotification(SOUND_EFFECT_VP_LAUNCH_WHEN_READY, 1);
}


void AnnounceWeaponsProgress(byte statusBefore, byte weaponIndex) {
  byte statusAfter = WeaponsStatus[CurrentPlayer][(weaponIndex-1)];
  if (statusAfter!=statusBefore) {
    if (statusAfter==100) {
      Audio.StopAllNotifications(3);
      QueueNotification(SOUND_EFFECT_VP_DEPTH_CHARGES+(weaponIndex-1), 3);
      QueueNotification(SOUND_EFFECT_VP_SPACER, 3);
      QueueNotification(SOUND_EFFECT_VP_100_PERCENT, 3);
    } else if (statusAfter>=95 && statusBefore<95) {
      Audio.StopAllNotifications(3);
      QueueNotification(SOUND_EFFECT_VP_DEPTH_CHARGES+(weaponIndex-1), 3);
      QueueNotification(SOUND_EFFECT_VP_SPACER, 3);
      QueueNotification(SOUND_EFFECT_VP_95_PERCENT, 3);
    } else if (statusAfter>=90 && statusBefore<90) {
      Audio.StopAllNotifications(3);
      QueueNotification(SOUND_EFFECT_VP_DEPTH_CHARGES+(weaponIndex-1), 3);
      QueueNotification(SOUND_EFFECT_VP_SPACER, 3);
      QueueNotification(SOUND_EFFECT_VP_90_PERCENT, 3);
    } else if (statusAfter>=75 && statusBefore<75) {
      Audio.StopAllNotifications(3);
      QueueNotification(SOUND_EFFECT_VP_DEPTH_CHARGES+(weaponIndex-1), 3);
      QueueNotification(SOUND_EFFECT_VP_SPACER, 3);
      QueueNotification(SOUND_EFFECT_VP_75_PERCENT, 3);
    } else if (statusAfter>=50 && statusBefore<50) {
      Audio.StopAllNotifications(3);
      QueueNotification(SOUND_EFFECT_VP_DEPTH_CHARGES+(weaponIndex-1), 3);
      QueueNotification(SOUND_EFFECT_VP_SPACER, 3);
      QueueNotification(SOUND_EFFECT_VP_50_PERCENT, 3);
    } else if (statusAfter>=25 && statusBefore<25) {
      Audio.StopAllNotifications(3);
      QueueNotification(SOUND_EFFECT_VP_DEPTH_CHARGES+(weaponIndex-1), 3);
      QueueNotification(SOUND_EFFECT_VP_SPACER, 3);
      QueueNotification(SOUND_EFFECT_VP_25_PERCENT, 3);
    }
  }
}




////////////////////////////////////////////////////////////////////////////
//
//  Diagnostics Mode
//
////////////////////////////////////////////////////////////////////////////

int RunDiagnosticsMode(int curState, boolean curStateChanged) {

  int returnState = curState;

  if (curStateChanged) {

/*
    char buf[256];
    boolean errorSeen;

    Serial.write("Testing Volatile RAM at IC13 (0x0000 - 0x0080): writing & reading... ");
    Serial.write("3 ");
    delay(500);
    Serial.write("2 ");
    delay(500);
    Serial.write("1 \n");
    delay(500);
    errorSeen = false;
    for (byte valueCount=0;valueCount<0xFF;valueCount++) {
      for (unsigned short address=0x0000;address<0x0080;address++) {
        WOS_DataWrite(address, valueCount);
      }
      for (unsigned short address=0x0000;address<0x0080;address++) {
        byte readValue = WOS_DataRead(address);
        if (readValue!=valueCount) {
          sprintf(buf, "Write/Read failure at address=0x%04X (expected 0x%02X, read 0x%02X)\n", address, valueCount, readValue);
          Serial.write(buf);
          errorSeen = true;
        }
        if (errorSeen) break;
      }
      if (errorSeen) break;
    }
    if (errorSeen) {
      Serial.write("!!! Error in Volatile RAM\n");
    }

    Serial.write("Testing Volatile RAM at IC16 (0x0080 - 0x0100): writing & reading... ");
    Serial.write("3 ");
    delay(500);
    Serial.write("2 ");
    delay(500);
    Serial.write("1 \n");
    delay(500);
    errorSeen = false;
    for (byte valueCount=0;valueCount<0xFF;valueCount++) {
      for (unsigned short address=0x0080;address<0x0100;address++) {
        WOS_DataWrite(address, valueCount);
      }
      for (unsigned short address=0x0080;address<0x0100;address++) {
        byte readValue = WOS_DataRead(address);
        if (readValue!=valueCount) {
          sprintf(buf, "Write/Read failure at address=0x%04X (expected 0x%02X, read 0x%02X)\n", address, valueCount, readValue);
          Serial.write(buf);
          errorSeen = true;
        }
        if (errorSeen) break;
      }
      if (errorSeen) break;
    }
    if (errorSeen) {
      Serial.write("!!! Error in Volatile RAM\n");
    }
    
    // Check the CMOS RAM to see if it's operating correctly
    errorSeen = false;
    Serial.write("Testing CMOS RAM: writing & reading... ");
    Serial.write("3 ");
    delay(500);
    Serial.write("2 ");
    delay(500);
    Serial.write("1 \n");
    delay(500);
    for (byte valueCount=0;valueCount<0x10;valueCount++) {
      for (unsigned short address=0x0100;address<0x0200;address++) {
        WOS_DataWrite(address, valueCount);
      }
      for (unsigned short address=0x0100;address<0x0200;address++) {
        byte readValue = WOS_DataRead(address);
        if ((readValue&0x0F)!=valueCount) {
          sprintf(buf, "Write/Read failure at address=0x%04X (expected 0x%02X, read 0x%02X)\n", address, valueCount, (readValue&0x0F));
          Serial.write(buf);
          errorSeen = true;
        }
        if (errorSeen) break;
      }
      if (errorSeen) break;
    }
    
    if (errorSeen) {
      Serial.write("!!! Error in CMOS RAM\n");
    }
    
    
    // Check the ROMs
    Serial.write("CMOS RAM dump... ");
    Serial.write("3 ");
    delay(500);
    Serial.write("2 ");
    delay(500);
    Serial.write("1 \n");
    delay(500);
    for (unsigned short address=0x0100;address<0x0200;address++) {
      if ((address&0x000F)==0x0000) {
        sprintf(buf, "0x%04X:  ", address);
        Serial.write(buf);
      }
//      WOS_DataWrite(address, address&0xFF);
      sprintf(buf, "0x%02X ", WOS_DataRead(address));
      Serial.write(buf);
      if ((address&0x000F)==0x000F) {
        Serial.write("\n");
      }
    }

*/

//    WOS_EnableSolenoidStack();
//    WOS_SetDisableFlippers(false);
        
  }

  return returnState;
}



////////////////////////////////////////////////////////////////////////////
//
//  Attract Mode
//
////////////////////////////////////////////////////////////////////////////

unsigned long AttractLastLadderTime = 0;
byte AttractLastLadderBonus = 0;
unsigned long AttractDisplayRampStart = 0;
byte AttractLastHeadMode = 255;
byte AttractLastPlayfieldMode = 255;
byte InAttractMode = false;


int RunAttractMode(int curState, boolean curStateChanged) {

  int returnState = curState;

  if (curStateChanged) {
    WOS_DisableSolenoidStack();
    WOS_TurnOffAllLamps();
    WOS_SetDisableFlippers(true);
    if (DEBUG_MESSAGES) {
      Serial.write("Entering Attract Mode\n\r");
    }
    AttractLastHeadMode = 0;
    AttractLastPlayfieldMode = 0;
    WOS_SetDisplayCredits(Credits, !FreePlayMode);
    for (byte count = 0;count < 4;count++) {
      WOS_SetLampState(LAMP_HEAD_PLAYER_1_UP + count, 0);
    }

    WOS_SetLampState(LAMP_HEAD_1_PLAYER, 0);
    WOS_SetLampState(LAMP_HEAD_2_PLAYERS, 0);
    WOS_SetLampState(LAMP_HEAD_3_PLAYERS, 0);
    WOS_SetLampState(LAMP_HEAD_4_PLAYERS, 0);

    if (WOS_ReadSingleSwitchState(SW_LEFT_SAUCER)) {
      WOS_PushToSolenoidStack(SOL_LEFT_SAUCER, 16, true);
      LeftSaucerKicked = millis();
    }
  
    if (WOS_ReadSingleSwitchState(SW_RIGHT_SAUCER)) {
      WOS_PushToTimedSolenoidStack(SOL_RIGHT_SAUCER, 16, CurrentTime + 1000, true);
      RightSaucerKicked = millis();
    }
  }
  
  MoveBallFromOutholeToRamp();
  
  // Alternate displays between high score and blank
  if (CurrentTime < 16000) {
    if (AttractLastHeadMode != 1) {
      ShowPlayerScores(0xFF, false, false);
      WOS_SetDisplayCredits(Credits, !FreePlayMode);
      WOS_SetDisplayBallInPlay(0, true);
    }
    AttractLastHeadMode = 1;
  } else if ((CurrentTime / 8000) % 2 == 0) {

    if (AttractLastHeadMode != 2) {
      WOS_SetLampState(LAMP_HEAD_HIGH_SCORE, 1, 0, 250);
      WOS_SetLampState(LAMP_HEAD_GAME_OVER, 0);
      LastTimeScoreChanged = CurrentTime;
    }
    AttractLastHeadMode = 2;
    ShowPlayerScores(0xFF, false, false, HighScore);
  } else {
    if (AttractLastHeadMode != 3) {
      if (CurrentTime < 32000) {
        for (int count = 0;count < 4;count++) {
          CurrentScores[count] = 0;
          CurrentAchievements[count] = 0;
        }
        CurrentNumPlayers = 0;
      }
      WOS_SetLampState(LAMP_HEAD_HIGH_SCORE, 0);
      WOS_SetLampState(LAMP_HEAD_GAME_OVER, 1);
      LastTimeScoreChanged = CurrentTime;
    }
    ShowPlayerScores(0xFF, false, false);

    AttractLastHeadMode = 3;
  }

  byte attractPlayfieldPhase = ((CurrentTime / 5000) % 11);
  if (attractPlayfieldPhase != AttractLastPlayfieldMode) {
  }

//  ShowLampAnimation(attractPlayfieldPhase, 40, CurrentTime, 14, false, false);

  if (attractPlayfieldPhase < 4) {
    SetGeneralIlluminationOn(true);
    ShowLampAnimation(attractPlayfieldPhase, 40, CurrentTime, 12, false, false);
  } else {
    SetGeneralIlluminationOn(false);
    ShowChaseScannerLamps((CurrentTime)%1000, (attractPlayfieldPhase-4) + GAME_MODE_CHASE_SPINNER);
  }


  byte switchHit;
  while ( (switchHit = WOS_PullFirstFromSwitchStack()) != SWITCH_STACK_EMPTY ) {
    if (switchHit == SW_CREDIT_RESET) {
      if (AddPlayer(true)) returnState = MACHINE_STATE_INIT_GAMEPLAY;
    }
    if (switchHit == SW_COIN_1 || switchHit == SW_COIN_2 || switchHit == SW_COIN_3) {
      AddCoinToAudit(SwitchToChuteNum(switchHit));
      AddCoin(SwitchToChuteNum(switchHit));
    }
    if (switchHit == SW_SELF_TEST_SWITCH && (CurrentTime - GetLastSelfTestChangedTime()) > 250) {
      returnState = MACHINE_STATE_TEST_BOOT;
      SetLastSelfTestChangedTime(CurrentTime);
    }
  }

  return returnState;
}





////////////////////////////////////////////////////////////////////////////
//
//  Game Play functions
//
////////////////////////////////////////////////////////////////////////////
byte CountBits(unsigned short intToBeCounted) {
  byte numBits = 0;

  for (byte count = 0;count < 16;count++) {
    numBits += (intToBeCounted & 0x01);
    intToBeCounted = intToBeCounted >> 1;
  }

  return numBits;
}


void SetGameMode(byte newGameMode) {
  GameMode = newGameMode;
  GameModeStartTime = 0;
  GameModeEndTime = 0;

  if (DEBUG_MESSAGES) {
    char buf[256];
    sprintf(buf, "Mode=%d\n", newGameMode);
    Serial.write(buf);
  }
}


unsigned long Trough1SwitchDown = 0;
unsigned long Trough2SwitchDown = 0;

byte CountBallsInTrough() {
  // WOS_ReadSingleSwitchState(SW_OUTHOLE) + 

  boolean ball1Steady = false;
  boolean ball2Steady = false;

  if (WOS_ReadSingleSwitchState(SW_BALL_1)) {
    if (Trough1SwitchDown!=0) {
      if (CurrentTime>(Trough1SwitchDown+500)) ball1Steady = true;
    } else {
      Trough1SwitchDown = CurrentTime;
    }
  } else {
    Trough1SwitchDown = 0;
  }

  if (WOS_ReadSingleSwitchState(SW_BALL_2)) {
    if (Trough2SwitchDown!=0) {
      if (CurrentTime>(Trough2SwitchDown+500)) ball2Steady = true;
    } else {
      Trough2SwitchDown = CurrentTime;
    }
  } else {
    Trough2SwitchDown = 0;
  }
/*
  (void)ball1Steady;
  (void)ball2Steady;


  byte numBalls = WOS_ReadSingleSwitchState(SW_BALL_1) + 
                  WOS_ReadSingleSwitchState(SW_BALL_2);
*/

  byte numBalls = ball1Steady ? 1 : 0;
  numBalls += ball2Steady ? 1 : 0;                  

  return numBalls;
}

void PrintLockStatus(byte reasonForCheck=0) {
  (void)reasonForCheck;
  if (DEBUG_MESSAGES) {
//    char buf[255];
//    sprintf(buf, "ML=0x%02X, LS=0x%02X, Numlocks=%d, NumBIP=%d (%d)\n", MachineLocks, LockStatus[CurrentPlayer], NumberOfBallsLocked, NumberOfBallsInPlay, reasonForCheck);
//    Serial.write(buf);
  }
}

boolean CheckLock(byte lockFlag, byte lockSwitch) {

  if (DEBUG_MESSAGES) {
    PrintLockStatus(1);
  }
  // We should do something with the lockSwitch to make sure our 
  // locks aren't out of sync
  (void)lockSwitch;
  
  if (MachineLocks&lockFlag) {
    return true;
  }

  return false;
}

byte GetNumPlayerLocks() {
  byte numPlayerLocks = 0;
  if (LockStatus[CurrentPlayer]&LOCK_STATUS_BLUE_LOCKED) numPlayerLocks += 1;
  if (LockStatus[CurrentPlayer]&LOCK_STATUS_GREEN_LOCKED) numPlayerLocks += 1;

  if (DEBUG_MESSAGES) { 
//    char buf[65];
//    sprintf(buf, "Number of locks = %d\n", numPlayerLocks);    
//    Serial.write(buf);
  }
  
  if (DEBUG_MESSAGES) PrintLockStatus(10);
  return numPlayerLocks;
}

boolean PutBallInPlay() {
  if (CountBallsInTrough()) {
    WOS_PushToTimedSolenoidStack(SOL_BALL_RAMP_THROWER, 25, CurrentTime + 1000);
    NumberOfBallsInPlay += 1;
    if (DEBUG_MESSAGES) PrintLockStatus(2);
    return true;
  }
  return false;
}


boolean AddABall(boolean ballSave = true) {
  if (NumberOfBallsInPlay>=TotalBallsLoaded) return false;
  boolean ballServed = false;

  if (CountBallsInTrough()) {
    WOS_PushToTimedSolenoidStack(SOL_BALL_RAMP_THROWER, 16, CurrentTime + 100);
    NumberOfBallsInPlay += 1;
    ballServed = true;

    if (DEBUG_MESSAGES) PrintLockStatus(3);

    if (ballSave) {
      if (BallSaveEndTime) BallSaveEndTime += 10000;
      else BallSaveEndTime = CurrentTime + 20000;
    }  
  }

  return ballServed;
}


void LockABall(byte activationFlag, byte availableFlag) {
  LockStatus[CurrentPlayer] |= activationFlag;
  LockStatus[CurrentPlayer] &= ~(availableFlag);

  if (!(MachineLocks & activationFlag)) {
    MachineLocks |= activationFlag;
    if (NumberOfBallsLocked<TotalBallsLoaded) NumberOfBallsLocked += 1;
    if (NumberOfBallsInPlay) NumberOfBallsInPlay -= 1;
  }
  
  if (DEBUG_MESSAGES) PrintLockStatus(4);
}


/*
void ActivateLock(byte lockToActivate) {
  LockStatus[CurrentPlayer] |= lockToActivate;
  LockStatus[CurrentPlayer] &= ~(lockToActivate/4);

  
}
*/

void SetLockStatusForStory() {

  LockStatus[CurrentPlayer] = 0;
  
  // Make sure machine locks are proper
  if (LeftSaucerLastState && !(MachineLocks&LOCK_STATUS_BLUE_LOCKED)) {
    if (DEBUG_MESSAGES) {
//      Serial.write("Blue machine lock corrected\n");
    }
    MachineLocks |= LOCK_STATUS_BLUE_LOCKED;
  }
  if (RightSaucerLastState && !(MachineLocks&LOCK_STATUS_GREEN_LOCKED)) {
    if (DEBUG_MESSAGES) {
//      Serial.write("Green machine lock corrected\n");
    }
    MachineLocks |= LOCK_STATUS_GREEN_LOCKED;
  }

  boolean leftLockReady = false;
  boolean rightLockReady = false;

  for (byte count=0; count<8; count++) {
    if (WeaponsStatus[CurrentPlayer][count]==100) {
      if (count%2) rightLockReady = true;
      else leftLockReady = true;
    }
  }

  if (LeftSaucerLastState) {
    if (leftLockReady) LockStatus[CurrentPlayer] |= LOCK_STATUS_BLUE_LOCKED;    
  } else {
    if (leftLockReady) LockStatus[CurrentPlayer] |= LOCK_STATUS_BLUE_READY;
  }

  if (RightSaucerLastState) {
    if (rightLockReady) LockStatus[CurrentPlayer] |= LOCK_STATUS_GREEN_LOCKED;    
  } else {
    if (rightLockReady) LockStatus[CurrentPlayer] |= LOCK_STATUS_GREEN_READY;
  }
}


void UpdateLockStatus(byte lockToCheck, byte readyFlag, byte switchToCheck) {

  boolean saucerState = LeftSaucerLastState;
  if (switchToCheck==SW_RIGHT_SAUCER) saucerState = RightSaucerLastState;

  // Check to see if a lock has been stolen.
  // If it was, mark the lock as ready.
  for (byte count=0; count<4; count++) {
    if (count<CurrentNumPlayers) {
      if (LockStatus[count]&lockToCheck && !saucerState) {
        LockStatus[count] &= ~lockToCheck;
        LockStatus[count] |= readyFlag;
      }
    }
  }
  if (DEBUG_MESSAGES) PrintLockStatus(5);
  
}


void ReleaseBallFromSaucer(byte kickSol) {
  if (kickSol==SOL_LEFT_SAUCER) {
    LeftSaucerLastState = false;
    LeftSaucerDownTime = 0;
    LeftSaucerUpTime = CurrentTime;
  } else {
    RightSaucerLastState = false;
    RightSaucerDownTime = 0;
    RightSaucerUpTime = CurrentTime;
  }
  WOS_PushToSolenoidStack(kickSol, 16, true);  
}



boolean ReleaseLock(byte lockToRelease, byte switchToCheck, byte solenoidToKick) {

  boolean saucerState = LeftSaucerLastState;
  if (switchToCheck==SW_RIGHT_SAUCER) saucerState = RightSaucerLastState;

  if (saucerState) {
    LockStatus[CurrentPlayer] &= ~lockToRelease;
    if (NumberOfBallsLocked) NumberOfBallsLocked -= 1;
    if (NumberOfBallsInPlay<TotalBallsLoaded) NumberOfBallsInPlay += 1;
//    WOS_PushToSolenoidStack(solenoidToKick, 16);
    ReleaseBallFromSaucer(solenoidToKick);

    MachineLocks &= ~lockToRelease;
    if (DEBUG_MESSAGES) PrintLockStatus(6);
    return true;
  }

  return false;
}


boolean KickUnearnedLock(byte lockToCheck, byte solenoidToKick) {

  if (MachineLocks & lockToCheck) {
    MachineLocks &= ~lockToCheck;

    if (!(LockStatus[CurrentPlayer]&lockToCheck)) {
      if (NumberOfBallsLocked) NumberOfBallsLocked -= 1;
      if (NumberOfBallsInPlay<TotalBallsLoaded) NumberOfBallsInPlay += 1;
      ReleaseBallFromSaucer(solenoidToKick);
//      WOS_PushToSolenoidStack(solenoidToKick, 16, true);
      return true;
    }

    if (DEBUG_MESSAGES) PrintLockStatus(7);
  }

  return false;  
}

void AddToBonus(byte leftBonus, byte rightBonus) {

  if (leftBonus) {
    BonusLeft[CurrentPlayer] += leftBonus;
    if (BonusLeft[CurrentPlayer]>MAX_DISPLAY_BONUS) {
      BonusLeft[CurrentPlayer] = MAX_DISPLAY_BONUS;
    } else {
      BonusLeftChanged = CurrentTime;
    }
  }

  if (rightBonus) {
    BonusRight[CurrentPlayer] += rightBonus;
    if (BonusRight[CurrentPlayer]>MAX_DISPLAY_BONUS) {
      BonusRight[CurrentPlayer] = MAX_DISPLAY_BONUS;
    } else {
      BonusRightChanged = CurrentTime;
    }
  }

}



void IncreaseBonusX() {
  boolean soundPlayed = false;
  if (BonusX[CurrentPlayer] < 10) {
    BonusX[CurrentPlayer] += 1;
    BonusXAnimationStart = CurrentTime;

    if (BonusX[CurrentPlayer] == 9) {
      BonusX[CurrentPlayer] = 10;
//      QueueNotification(SOUND_EFFECT_VP_BONUSX_MAX, 2);
    } else {
      QueueNotification(SOUND_EFFECT_VP_BONUS_X_INCREASED, 1);
    }
  }

  if (!soundPlayed) {
    //    PlaySoundEffect(SOUND_EFFECT_BONUS_X_INCREASED);
  }

}

unsigned long GameStartNotificationTime = 0;
boolean WaitForBallToReachOuthole = false;

int InitGamePlay(boolean curStateChanged) {
  
  if (curStateChanged) {
    WOS_DisableSolenoidStack();
    WOS_SetDisableFlippers(true);
    WOS_TurnOffAllLamps();
    SetGeneralIlluminationOn(true);
    CurrentNumPlayers = 1;
    WOS_SetLampState(LAMP_HEAD_1_PLAYER, 1, 0, 500);
    WOS_SetLampState(LAMP_HEAD_2_PLAYERS, 0);
    WOS_SetLampState(LAMP_HEAD_3_PLAYERS, 0);
    WOS_SetLampState(LAMP_HEAD_4_PLAYERS, 0);
    CurrentPlayer = 0;
    CurrentBallInPlay = 0;
    for (byte count=0; count<4; count++) {
      CurrentScores[count] = 0;
    }
    WOS_SetDisplayBallInPlay(0, true);
    ShowPlayerScores(0xFF, false, false);
    GameStartNotificationTime = CurrentTime;
  }

  if (CountBallsInTrough()<2) {
    if (CurrentTime>(LeftSaucerKicked+1000) && WOS_ReadSingleSwitchState(SW_LEFT_SAUCER)) {
      WOS_PushToSolenoidStack(SOL_LEFT_SAUCER, 16, true);
      LeftSaucerKicked = CurrentTime;
    }
  
    if (CurrentTime>(RightSaucerKicked+1000) && WOS_ReadSingleSwitchState(SW_RIGHT_SAUCER)) {
      WOS_PushToSolenoidStack(SOL_RIGHT_SAUCER, 16, true);
      RightSaucerKicked = CurrentTime;
    }

    MoveBallFromOutholeToRamp();

    if (CurrentTime>(GameStartNotificationTime+5000)) {
      GameStartNotificationTime = CurrentTime;
      QueueNotification(SOUND_EFFECT_VP_BALL_MISSING, 10);
    }
    
    return MACHINE_STATE_INIT_GAMEPLAY;
  }
  

  // The start button has been hit only once to get
  // us into this mode, so we assume a 1-player game
  // at the moment
  WOS_EnableSolenoidStack();
  WOS_SetCoinLockout((Credits >= MaximumCredits) ? true : false);

  // Reset displays & game state variables
  for (int count = 0;count < 4;count++) {
    // Initialize game-specific variables
    BonusX[count] = 1;
    BonusTimeAccumulated[count] = 0;
    for (byte popCount=0; popCount<3; popCount++) TotalPopHits[count][popCount] = 0;
    HoldoverAwards[count] = 0;
    BonusLeft[count] = 0;
    BonusRight[count] = 0;
    CombosAchieved[count] = 0;
    CurrentWeaponsEngaged[count] = 0;
    for (int weapon=0;weapon<8;weapon++) WeaponsStatus[count][weapon] = 0;
    TopLaneLevel[count] = 0;    
    TopLaneStatus[count] = 0;
    StandupStatus[count] = 0;
    StandupLevel[count] = 0;
    LockStatus[count] = 0;
    CurrentAchievements[count] = 0;
    Drop6Clears[count] = 0;
    Drop5Clears[count] = 0;
    LaneToTopValue[count] = 0;
    StoriesEngaged[count] = 0;
    StoriesFinished[count] = 0;
    DoubleTimerBonus[count] = false;
  }

  SamePlayerShootsAgain = false;
  CurrentBallInPlay = 1;
  MachineLocks = 0;
  NumberOfBallsInPlay = 0;
  NumberOfBallsLocked = 0;
  LaneToTopLastHit = 0;

  // re-zero scores in case anything was hit during 
  // "ball missing"
  for (byte count=0; count<4; count++) {
    CurrentScores[count] = 0;
  }
  ShowPlayerScores(0xFF, false, false);

  return MACHINE_STATE_INIT_NEW_BALL;
}


int InitNewBall(bool curStateChanged, byte playerNum, int ballNum) {

  // If we're coming into this mode for the first time
  // then we have to do everything to set up the new ball
  if (curStateChanged) {
    WOS_TurnOffAllLamps();
    BallFirstSwitchHitTime = 0;

    // Choose a random lane for the skill shot
    if (playerNum == 0) SkillShotLane = 0x01<<(CurrentTime % 3);

    WOS_SetDisableFlippers(false);
    WOS_EnableSolenoidStack();
    WOS_SetDisplayCredits(Credits, !FreePlayMode);
//    if (CurrentNumPlayers > 1 && (ballNum != 1 || playerNum != 0) && !SamePlayerShootsAgain) AlertPlayerUp(playerNum);
    SamePlayerShootsAgain = false;

    WOS_SetDisplayBallInPlay(ballNum);
    WOS_SetLampState(LAMP_HEAD_TILT, 0);

    if (BallSaveNumSeconds > 0) {
      WOS_SetLampState(LAMP_SHOOT_AGAIN, 1, 0, 500);
      WOS_SetLampState(LAMP_HEAD_SAME_PLAYER_SHOOTS_AGAIN, 1, 0, 500);
    }

    BallSaveUsed = false;
    BallTimeInTrough = 0;
    NumTiltWarnings = 0;
    LastTiltWarningTime = 0;

    // Initialize game-specific start-of-ball lights & variables
    SetGameMode(GAME_MODE_PRE_PLUNGE_CHOICE);

    ExtraBallCollected = false;
    SpecialCollected = false;

    PlayfieldMultiplier = 1;
    TotalScoreMultiplier = 1;
    PlayfieldMultiplierExpiration = 0;
    ScoreAdditionAnimation = 0;
    ScoreAdditionAnimationStartTime = 0;
    BonusXAnimationStart = 0;
    LastSpinnerHit = 0;

    if (!(HoldoverAwards[CurrentPlayer]&HOLDOVER_BONUS)) {
      BonusLeft[CurrentPlayer] = 1;
      BonusRight[CurrentPlayer] = 1;
    }
    if (!(HoldoverAwards[CurrentPlayer]&HOLDOVER_BONUS_X)) {
      BonusX[CurrentPlayer] = 1;
    }
    if (!(HoldoverAwards[CurrentPlayer]&HOLDOVER_MULTIBALL_TIMER)) {
      BonusTimeAccumulated[CurrentPlayer] = 0;
      DoubleTimerBonus[CurrentPlayer] = false;
    }
    BonusLeftChanged = 0;
    BonusRightChanged = 0;
    WeaponsStatusShownTime = 0;
    SkillShotLaneHitTime = 0;

    SpinnerFrenzyEndTime = 0;
    BallSaveEndTime = 0;
    IdleMode = IDLE_MODE_NONE;

    CurrentWeaponSelected = 0;
    TopLanesCompletedTime = 0;
    WeaponChangedOnce = false;
    HatchlingsProgress = 0;

    LastRightPopHit = 0;
    LastUpperPopHit = 0;
    LastLowerPopHit = 0;
    for (byte count=0; count<3; count++) LastTopLaneHit[count] = CurrentTime;
    for (byte count=0; count<4; count++) LastInlaneHit[count] = CurrentTime;
    LastOutlaneHit = CurrentTime;
    WeaponsProgressChangedTime = 0;

    if (WOS_ReadSingleSwitchState(SW_RIGHT_FLIPPER)) RightFlipperDownTime = CurrentTime;
    else RightFlipperDownTime = 0;

//    for (byte count = 0;count < NUM_BALL_SEARCH_SOLENOIDS;count++) {
//      BallSearchSolenoidFireTime[count] = 0;
//    }

    // Reset Drop Targets
    DropTargets6.ResetDropTargets(CurrentTime + 200, true);
    DropTargets5.ResetDropTargets(CurrentTime + 400, true);

    UpdateLockStatus(LOCK_STATUS_BLUE_LOCKED, LOCK_STATUS_BLUE_READY, SW_LEFT_SAUCER);
    UpdateLockStatus(LOCK_STATUS_GREEN_LOCKED, LOCK_STATUS_GREEN_READY, SW_RIGHT_SAUCER);
    PutBallInPlay();

    PlayBackgroundSong(SOUND_EFFECT_BACKGROUND_SONG_1 + ((CurrentTime / 10) % NUM_BACKGROUND_SONGS));
//    Audio.PlaySoundCardWhenPossible(17 * 256, CurrentTime);
  }

  // We should only consider the ball initialized when
  // the ball is no longer triggering the SW_OUTHOLE
  if (CountBallsInTrough()==(TotalBallsLoaded-NumberOfBallsLocked)) {
    return MACHINE_STATE_INIT_NEW_BALL;
  } else {
    return MACHINE_STATE_NORMAL_GAMEPLAY;
  }

  LastTimeThroughLoop = CurrentTime;
}





void AnnounceStatus() {
  /*
        if (TicksCountedTowardsStatus > 68000) {
          IdleMode = IDLE_MODE_NONE;
          TicksCountedTowardsStatus = 0;
        } else if (TicksCountedTowardsStatus > 59000) {
          if (IdleMode != IDLE_MODE_BALL_SEARCH) {
            BallSearchSolenoidToTry = 0;
            BallSearchNextSolenoidTime = CurrentTime - 1;
          }
          if (CurrentTime > BallSearchNextSolenoidTime) {
            // Fire off a solenoid
            BallSearchSolenoidFireTime[BallSearchSolenoidToTry] = CurrentTime;
            WOS_PushToSolenoidStack(BallSearchSols[BallSearchSolenoidToTry], 10);
            BallSearchSolenoidToTry += 1;
            if (BallSearchSolenoidToTry >= NUM_BALL_SEARCH_SOLENOIDS) BallSearchSolenoidToTry = 0;
            BallSearchNextSolenoidTime = CurrentTime + 500;
          }
          IdleMode = IDLE_MODE_BALL_SEARCH;
        } else if (TicksCountedTowardsStatus > 52000) {
          if (WizardGoals[CurrentPlayer]&WIZARD_GOAL_SHIELD) {
            TicksCountedTowardsStatus = 59001;
          } else {
            if (IdleMode != IDLE_MODE_ADVERTISE_SHIELD) QueueNotification(SOUND_EFFECT_VP_ADVERTISE_SHIELD, 1);
            IdleMode = IDLE_MODE_ADVERTISE_SHIELD;
          }
        } else if (TicksCountedTowardsStatus > 45000) {
          if (WizardGoals[CurrentPlayer]&WIZARD_GOAL_SPINS) {
            TicksCountedTowardsStatus = 52001;
          } else {
            if (IdleMode != IDLE_MODE_ADVERTISE_SPINS) QueueNotification(SOUND_EFFECT_VP_ADVERTISE_SPINS, 1);
            IdleMode = IDLE_MODE_ADVERTISE_SPINS;
          }
        } else if (TicksCountedTowardsStatus > 38000) {
          if (WizardGoals[CurrentPlayer]&WIZARD_GOAL_7_NZ) {
            TicksCountedTowardsStatus = 45001;
          } else {
            if (IdleMode != IDLE_MODE_ADVERTISE_NZS) QueueNotification(SOUND_EFFECT_VP_ADVERTISE_NZS, 1);
            IdleMode = IDLE_MODE_ADVERTISE_NZS;
            ShowLampAnimation(0, 40, CurrentTime, 11, false, false);
            specialAnimationRunning = true;
          }
        } else if (TicksCountedTowardsStatus > 31000) {
          if (WizardGoals[CurrentPlayer]&WIZARD_GOAL_POP_BASES) {
            TicksCountedTowardsStatus = 38001;
          } else {
            if (IdleMode != IDLE_MODE_ADVERTISE_BASES) QueueNotification(SOUND_EFFECT_VP_ADVERTISE_BASES, 1);
            IdleMode = IDLE_MODE_ADVERTISE_BASES;
          }
        } else if (TicksCountedTowardsStatus > 24000) {
          if (WizardGoals[CurrentPlayer]&WIZARD_GOAL_COMBOS) {
            TicksCountedTowardsStatus = 31001;
          } else {
            if (IdleMode != IDLE_MODE_ADVERTISE_COMBOS) {
              byte countBits = CountBits(CombosAchieved[CurrentPlayer]);
              if (countBits==0) QueueNotification(SOUND_EFFECT_VP_ADVERTISE_COMBOS, 1);
              else if (countBits>0) QueueNotification(SOUND_EFFECT_VP_FIVE_COMBOS_LEFT+(countBits-1), 1);
            }
            IdleMode = IDLE_MODE_ADVERTISE_COMBOS;
          }
        } else if (TicksCountedTowardsStatus > 17000) {
          if (WizardGoals[CurrentPlayer]&WIZARD_GOAL_INVASION) {
            TicksCountedTowardsStatus = 24001;
          } else {
            if (IdleMode != IDLE_MODE_ADVERTISE_INVASION) QueueNotification(SOUND_EFFECT_VP_ADVERTISE_INVASION, 1);
            IdleMode = IDLE_MODE_ADVERTISE_INVASION;
          }
        } else if (TicksCountedTowardsStatus > 10000) {
          if (WizardGoals[CurrentPlayer]&WIZARD_GOAL_BATTLE) {
            TicksCountedTowardsStatus = 17001;
          } else {
            if (IdleMode != IDLE_MODE_ADVERTISE_BATTLE) QueueNotification(SOUND_EFFECT_VP_ADVERTISE_BATTLE, 1);
            IdleMode = IDLE_MODE_ADVERTISE_BATTLE;
          }
        } else if (TicksCountedTowardsStatus > 7000) {
          int goalCount = (int)(CountBits((WizardGoals[CurrentPlayer] & ~UsedWizardGoals[CurrentPlayer]))) + NumCarryWizardGoals[CurrentPlayer];
          if (GoalsUntilWizard==0) {
            TicksCountedTowardsStatus = 10001;
          } else {
            byte goalsRemaining = GoalsUntilWizard-(goalCount%GoalsUntilWizard);
            if (goalCount<0) goalsRemaining = (byte)(-1*goalCount);
            
            if (IdleMode != IDLE_MODE_ANNOUNCE_GOALS) {
              QueueNotification(SOUND_EFFECT_VP_ONE_GOAL_FOR_ENEMY-(goalsRemaining-1), 1);
              if (DEBUG_MESSAGES) {
                char buf[256];
                sprintf(buf, "Goals remaining = %d, Goals Until Wiz = %d, goalcount = %d, LO=%d, WizG=0x%04X\n", goalsRemaining, GoalsUntilWizard, goalCount, WizardGoals[CurrentPlayer], NumCarryWizardGoals[CurrentPlayer]);
                Serial.write(buf);
              }
            }
            IdleMode = IDLE_MODE_ANNOUNCE_GOALS;
            ShowLampAnimation(2, 40, CurrentTime, 11, false, false);
            specialAnimationRunning = true;
          }
        }
*/
}



void UpdateDropTargets() {
  DropTargets6.Update(CurrentTime);
  DropTargets5.Update(CurrentTime);
}

boolean CheckForFoundLock(byte weaponHitIndex) {
  boolean announcementDone = false;
  byte lockBit = LOCK_STATUS_BLUE_LOCKED;
  byte lockReadyBit = LOCK_STATUS_BLUE_READY;
  int readyNotification = SOUND_EFFECT_VP_BLUE_LOCK_READY;
  int lockNotification = SOUND_EFFECT_VP_BLUE;
  
  if ((weaponHitIndex%2)==0) {
    lockBit = LOCK_STATUS_GREEN_LOCKED;
    lockReadyBit = LOCK_STATUS_GREEN_READY;
    readyNotification = SOUND_EFFECT_VP_GREEN_LOCK_READY;
    lockNotification = SOUND_EFFECT_VP_GREEN;
  }

  if (MachineLocks & lockBit) {
    // A ball is already locked in this spot
    LockStatus[CurrentPlayer] |= (lockReadyBit|lockBit);
    QueueNotification(lockNotification, 4);
    QueueNotification(SOUND_EFFECT_VP_LOCKED, 4);
    announcementDone = true;
  } else if (!(LockStatus[CurrentPlayer] & lockReadyBit)) {
    LockStatus[CurrentPlayer] |= lockReadyBit;
    QueueNotification(readyNotification, 4);
    announcementDone = true;
  }

  return announcementDone;
}

void SetQualifiedGameLocks(byte weaponHitIndex) {
  if (StoriesEngaged[CurrentPlayer]&STORY_DEFEND) {

    // any hit opens a lock for defense
    byte lockBit = LOCK_STATUS_BLUE_LOCKED;
    byte lockReadyBit = LOCK_STATUS_BLUE_READY;
    if ((weaponHitIndex%2)==0) {
      lockBit = LOCK_STATUS_GREEN_LOCKED;
      lockReadyBit = LOCK_STATUS_GREEN_READY;
    }
    
    // We're in the defend mode:
    //  If we have a blue weapon, turn on that lock unless it's already on
    if (MachineLocks & lockBit) {
      // A ball is already locked in this spot, so we open the opposite lock
      LockStatus[CurrentPlayer] |= (lockReadyBit^(LOCK_STATUS_GREEN_READY|LOCK_STATUS_BLUE_READY));
    } else {
      LockStatus[CurrentPlayer] |= lockReadyBit;
    }
    
  } else if (StoriesEngaged[CurrentPlayer]&STORY_ATTACK) {

    PrintLockStatus(8);
    if (WeaponsStatus[CurrentPlayer][weaponHitIndex-1]==100) {  
      // any hit opens a lock for defense
      byte lockBit = LOCK_STATUS_BLUE_LOCKED;
      byte lockReadyBit = LOCK_STATUS_BLUE_READY;
      if ((weaponHitIndex%2)==0) {
        lockBit = LOCK_STATUS_GREEN_LOCKED;
        lockReadyBit = LOCK_STATUS_GREEN_READY;
      }
  
      if (!(MachineLocks&lockBit)) {
        LockStatus[CurrentPlayer] |= lockReadyBit;      
      }
      PrintLockStatus(9);
    }
  }
}


byte WeaponIndexToBitmask(byte weaponIndex) {
  return (0x01<<(weaponIndex-1));
}

byte GetMaximumWeaponStrength(boolean rightWeapon) {
  byte maxWeaponStrength = 0;

  for (byte count=0; count<4; count++) {
    if (WeaponsStatus[CurrentPlayer][count*2 + (byte)rightWeapon]>maxWeaponStrength) {
      maxWeaponStrength = WeaponsStatus[CurrentPlayer][count*2 + (byte)rightWeapon];
    }
  }

  return maxWeaponStrength;
}

byte GetNextUnselectedWeapon(boolean pickFromLeftWeapons, boolean pickFromRightWeapons) {
  byte nextWeapon = WEAPON_INDEX_DEPTH_CHARGES;
  if (!pickFromLeftWeapons && pickFromRightWeapons) nextWeapon = WEAPON_INDEX_CANNONS;

  if (pickFromLeftWeapons && pickFromRightWeapons) {
    for (byte count=0; count<8; count++) {
      if (CurrentWeaponsEngaged[CurrentPlayer]&WeaponIndexToBitmask(nextWeapon)) nextWeapon += 1;
      else break;  
    }
  } else {
    for (byte count=0; count<4; count++) {
      if (CurrentWeaponsEngaged[CurrentPlayer]&WeaponIndexToBitmask(nextWeapon)) nextWeapon += 2;
      else break;  
    }
  }
  if (nextWeapon>8) nextWeapon -= 8;

  return nextWeapon;
}



void SpreadHatchlings() {
  unsigned long lastProgress = HatchlingsProgress;
  
  if (!(HatchlingsProgress & HATCHLING_SPECIAL_RIGHT) && (HatchlingsProgress & HATCHLING_EXTRA)) {
    HatchlingsProgress |= HATCHLING_SPECIAL_RIGHT;
  } else if (!(HatchlingsProgress & HATCHLING_EXTRA) && (HatchlingsProgress & HATCHLING_50K_RIGHT)) {
    HatchlingsProgress |= HATCHLING_EXTRA;
  } else if (!(HatchlingsProgress & HATCHLING_50K_RIGHT) && (HatchlingsProgress & HATCHLING_RIGHT_POP)) {
    HatchlingsProgress |= HATCHLING_50K_RIGHT;
  }  
  
  if (!(HatchlingsProgress & HATCHLING_SPECIAL) && (HatchlingsProgress & HATCHLING_50K)) {
    HatchlingsProgress |= HATCHLING_SPECIAL;
  } else if (!(HatchlingsProgress & HATCHLING_50K) && (HatchlingsProgress & HATCHLING_30K)) {
    HatchlingsProgress |= HATCHLING_50K;
  } else if (!(HatchlingsProgress & HATCHLING_30K) && (HatchlingsProgress & HATCHLING_20K)) {
    HatchlingsProgress |= HATCHLING_30K;
  } else if (!(HatchlingsProgress & HATCHLING_20K) && (HatchlingsProgress & HATCHLING_10K)) {
    HatchlingsProgress |= HATCHLING_20K;
  } else if (!(HatchlingsProgress & HATCHLING_10K) && (HatchlingsProgress & HATCHLING_SPINNER)) {
    HatchlingsProgress |= HATCHLING_10K;
  }

  if (!(HatchlingsProgress & HATCHLING_LITES_OUT) && (HatchlingsProgress & HATCHLING_UPPER_POP)) {
    HatchlingsProgress |= HATCHLING_LITES_OUT;
  }
    
  if (!(HatchlingsProgress & HATCHLING_RIGHT_POP) && (HatchlingsProgress & (HATCHLING_ZONE_2|HATCHLING_ZONE_6))) {
    HatchlingsProgress |= HATCHLING_RIGHT_POP;
  }
  if (!(HatchlingsProgress & HATCHLING_SPINNER) && (HatchlingsProgress & (HATCHLING_ZONE_3|HATCHLING_ZONE_4))) {
    HatchlingsProgress |= HATCHLING_SPINNER;
  }
  if (!(HatchlingsProgress & HATCHLING_UPPER_POP) && (HatchlingsProgress & (HATCHLING_ZONE_1|HATCHLING_ZONE_5))) {
    HatchlingsProgress |= HATCHLING_UPPER_POP;
  }

  if (lastProgress==HatchlingsProgress) {
    if (!(HatchlingsProgress & HATCHLING_SPINNER) && (HatchlingsProgress & HATCHLING_LITES_OUT)) {
      HatchlingsProgress |= HATCHLING_SPINNER;
    }

    if (!(HatchlingsProgress & HATCHLING_UPPER_POP) && (HatchlingsProgress & HATCHLING_SPECIAL_RIGHT)) {
      HatchlingsProgress |= HATCHLING_UPPER_POP;
    }
    
    if (!(HatchlingsProgress & HATCHLING_RIGHT_POP) && (HatchlingsProgress & HATCHLING_SPECIAL)) {
      HatchlingsProgress |= HATCHLING_RIGHT_POP;
    }
  }
}


byte GameModeStage;
boolean DisplaysNeedRefreshing = false;
unsigned long LastTimePromptPlayed = 0;
unsigned long ChasePromptPlayed = 0;
unsigned long LastEggAnnouncement = 0;
unsigned long AccumulatedPlayTime = 0;
unsigned long OutholeCheckTime = 0;

#define NUM_GI_FLASH_SEQUENCE_ENTRIES 25
byte GIFlashIndex = 0;
unsigned long GIFlashTimer = 0;
unsigned int GIFlashChangeState[NUM_GI_FLASH_SEQUENCE_ENTRIES] = {  0, 60, 120, 180, 240, 300, 
                                                                    1000, 1060, 1120, 1180, 1240, 1300, 
                                                                    2000, 2060, 2120, 2180, 2240, 2300, 
                                                                    3000, 3060, 3120, 3180, 3240, 3300, 4000};

// This function manages all timers, flags, and lights
int ManageGameMode() {
  int returnState = MACHINE_STATE_NORMAL_GAMEPLAY;

  boolean specialAnimationRunning = false;

  UpdateDropTargets();
  
  TotalScoreMultiplier = PlayfieldMultiplier * ((unsigned long)NumberOfBallsInPlay);
  
  if (CurrentTime>(LastSwitchHitTime+1500)) TimersPaused = true;
  else TimersPaused = false;

  if (GIOverrideEndTime) {
    if (CurrentTime>GIOverrideEndTime) {
      GIOverrideEndTime = 0;
      SetGeneralIlluminationOn(RequestedGIState);    
    }
  }
  
  if (WeaponsProgressChangedTime) {
    if (CurrentTime>(WeaponsProgressChangedTime+5000)) {
      WeaponsProgressChangedTime = 0;
      WOS_SetDisplayCredits(Credits, !FreePlayMode);
    } else if (LastWeaponChanged<9) {
      byte flashPhase = ((CurrentTime-WeaponsProgressChangedTime)/150)%2;
      byte valueToShow = WeaponsStatus[CurrentPlayer][LastWeaponChanged-1];
      if (valueToShow>99) valueToShow = 99;
      if (flashPhase) WOS_SetDisplayCredits(valueToShow, true, false);
      else WOS_SetDisplayCredits(Credits, false);
    }
  }

  byte lockFlag, readyFlag, oppositeLockFlag, kickSol;

  switch ( GameMode ) {
    case GAME_MODE_PRE_PLUNGE_CHOICE:
      if (GameModeStartTime == 0) {
        GameModeStartTime = CurrentTime;
        GameModeEndTime = 0;
        LastTimePromptPlayed = CurrentTime;
        CurrentWeaponSelected = GetNextUnselectedWeapon(true, true);
        GameModeStage = 0;
        SetGeneralIlluminationOn(false);
      }

      if (WeaponsStatusShownTime) {
        if (CurrentTime>(WeaponsStatusShownTime+5000)) {
          WeaponsStatusShownTime = 0;
          ShowPlayerScores(0xFF, false, false);
        }
      }

      if (!WeaponChangedOnce && CurrentTime > (LastTimePromptPlayed + 10000) && GameModeStage==0) {
        LastTimePromptPlayed = CurrentTime;
        GameModeStage = 1;
        QueueNotification(SOUND_EFFECT_VP_SELECT_WEAOPNS_RIGHT_FLIPPER, 1);
      }

      if (GameModeStage==1 && CurrentTime>(LastTimePromptPlayed+10000)) {
        QueueNotification(SOUND_EFFECT_VP_WOKE_THE_CREATURE_1 + CurrentTime%5, 1);
        LastTimePromptPlayed = CurrentTime;
        GameModeStage = 2;
      }

      if (CurrentWeaponSelected==WEAPON_INDEX_RAIL_GUNS) {
        ShowLampAnimation(4, 20, CurrentTime, LAMP_ANIMATION_STEPS-5, false, false);
        specialAnimationRunning = true;
      }

      if (BallFirstSwitchHitTime != 0) {
        SetGameMode(GAME_MODE_BUILD_WEAPONS);
        SuperSkillShotReady = false;
      }
      break;
    case GAME_MODE_SKILL_SHOT:
      if (GameModeStartTime == 0) {
        GameModeStartTime = CurrentTime;
        GameModeEndTime = 0;
        LastTimePromptPlayed = CurrentTime;
        SetGeneralIlluminationOn(true);
      }

      if (BallFirstSwitchHitTime != 0) {
        SetGameMode(GAME_MODE_BUILD_WEAPONS);
        SuperSkillShotReady = false;
      }

      if (CurrentTime > (LastTimePromptPlayed + 8000)) {
        LastTimePromptPlayed = CurrentTime;
        AlertPlayerUp(CurrentPlayer);
      }
      break;


    case GAME_MODE_BUILD_WEAPONS:
      // If this is the first time in this mode
      if (GameModeStartTime == 0) {
        GameModeStartTime = CurrentTime;
        AccumulatedPlayTime = 0;
        DisplaysNeedRefreshing = true;
        TicksCountedTowardsStatus = 0;
        IdleMode = IDLE_MODE_NONE;
        
        if ((StoriesFinished[CurrentPlayer]&STORY_DEFEND)==0) {
          StoriesEngaged[CurrentPlayer] = STORY_DEFEND;
        } else {
          StoriesEngaged[CurrentPlayer] = STORY_ATTACK;
        }
        SetLockStatusForStory();

        if (CurrentWeaponSelected) {
          if (DEBUG_MESSAGES) {
//            char buf[129];
//            sprintf(buf, "Sel=%d, eng=0x%02X, bitm=0x%02X\n", CurrentWeaponSelected, CurrentWeaponsEngaged[CurrentPlayer], WeaponIndexToBitmask(CurrentWeaponSelected));
//            Serial.write(buf);
          }
          if (WeaponIndexToBitmask(CurrentWeaponSelected) & CurrentWeaponsEngaged[CurrentPlayer]) {
            RegisterWeaponsHit(CurrentWeaponSelected, WeaponIndexToBitmask(CurrentWeaponSelected), 100);
            if (DEBUG_MESSAGES) {
//              char buf[64];
//              sprintf(buf, "Reg=%d, 0x%02X\n", CurrentWeaponSelected, WeaponIndexToBitmask(CurrentWeaponSelected));
//              Serial.write(buf);
            }
            QueueNotification(SOUND_EFFECT_VP_100_PERCENT, 10);
          } else {
            if (DEBUG_MESSAGES) {
              Serial.write("Not already engaged\n");
            }
          }
          CurrentWeaponsEngaged[CurrentPlayer] |= WeaponIndexToBitmask(CurrentWeaponSelected);
        }
        CurrentWeaponSelected = 0;
        if (DEBUG_MESSAGES) {
          Serial.write("Entering unstructured play\n");
        }
        SetGeneralIlluminationOn(true);
        GameModeStage = 0;
        LastTimePromptPlayed = CurrentTime;
        BuildWeaponsLastModeHit = CurrentTime;
        LastEggAnnouncement = 0;
      }

      if (CurrentTime<(BuildWeaponsLastModeHit+500)) {
        AccumulatedPlayTime = 0;
      } else {
        if (!TimersPaused && (StoriesFinished[CurrentPlayer]&STORY_EGGS)==0) {
          AccumulatedPlayTime += (CurrentTime-LastTimeThroughLoop);
          if (LastEggAnnouncement==0 && AccumulatedPlayTime>(EGGS_IDLE_TIME-5000)) {
            LastEggAnnouncement = CurrentTime;
            QueueNotification(SOUND_EFFECT_VP_GOING_TO_LAY_EGGS, 2);
          }
          if (AccumulatedPlayTime>EGGS_IDLE_TIME) {
            SetGameMode(GAME_MODE_EGGS_START);
          }
        }
      }

      if (StoriesEngaged[CurrentPlayer]==STORY_DEFEND && CurrentTime>(LastTimePromptPlayed+7500)) {
        if ((LockStatus[CurrentPlayer]&(LOCK_STATUS_BLUE_READY|LOCK_STATUS_GREEN_READY))==0) {
          if (GameModeStage==0) {
            GameModeStage = 1;
            QueueNotification(SOUND_EFFECT_VP_ACTIVATE_THE_WEAPON, 1);
            LastTimePromptPlayed = CurrentTime;
          } else if (GameModeStage==1) {
            GameModeStage = 2;
            if (CurrentWeaponsEngaged[CurrentPlayer]&WEAPONS_MODE_DEPTH_CHARGES) QueueNotification(SOUND_EFFECT_VP_ACTIVATE_6_DROP, 1);
            else if (CurrentWeaponsEngaged[CurrentPlayer]&WEAPONS_MODE_CANNONS) QueueNotification(SOUND_EFFECT_VP_ACTIVATE_POP, 1);
            else if (CurrentWeaponsEngaged[CurrentPlayer]&WEAPONS_MODE_HARPOONS) QueueNotification(SOUND_EFFECT_VP_ACTIVATE_STANDUP, 1);
            else if (CurrentWeaponsEngaged[CurrentPlayer]&WEAPONS_MODE_AIR_ASSAULT) QueueNotification(SOUND_EFFECT_VP_ACTIVATE_TOP_LANE, 1);
            else if (CurrentWeaponsEngaged[CurrentPlayer]&WEAPONS_MODE_RAIL_GUNS) QueueNotification(SOUND_EFFECT_VP_ACTIVATE_BULLSEYE, 1);
            else if (CurrentWeaponsEngaged[CurrentPlayer]&WEAPONS_MODE_TORPEDOES) QueueNotification(SOUND_EFFECT_VP_ACTIVATE_5_DROP, 1);
            else if (CurrentWeaponsEngaged[CurrentPlayer]&WEAPONS_MODE_BLUE_LASER) QueueNotification(SOUND_EFFECT_VP_ACTIVATE_BLUE, 1);
            else if (CurrentWeaponsEngaged[CurrentPlayer]&WEAPONS_MODE_GREEN_LASER) QueueNotification(SOUND_EFFECT_VP_ACTIVATE_GREEN, 1);
                        
            LastTimePromptPlayed = CurrentTime;
          }
        }
      }

      if ((StoriesFinished[CurrentPlayer]&ALL_BASIC_STORIES)==ALL_BASIC_STORIES) {
        SetGameMode(GAME_MODE_VICTORY_START);
      }

      if (!TimersPaused && NumberOfBallsInPlay==2) {
        WOS_SetLampState(LAMP_HEAD_RIGHT_TIMER, 1, 0, 250);
        BonusTimeAccumulated[CurrentPlayer] += (CurrentTime-LastTimeThroughLoop);
        if (BonusTimeAccumulated[CurrentPlayer]>99000) BonusTimeAccumulated[CurrentPlayer] = 99000;
        WOS_SetDisplayBallInPlay(BonusTimeAccumulated[CurrentPlayer]/1000, ((CurrentTime/250)%2), true);
      } else {
        WOS_SetLampState(LAMP_HEAD_RIGHT_TIMER, 0);
        if (BonusTimeAccumulated[CurrentPlayer]) WOS_SetDisplayBallInPlay(BonusTimeAccumulated[CurrentPlayer]/1000);
        else WOS_SetDisplayBallInPlay(CurrentBallInPlay);
      }

      if (TimersPaused && IdleModeOn) {
        TicksCountedTowardsStatus += (CurrentTime - LastTimeThroughLoop);
        AnnounceStatus();
      } else {
        TicksCountedTowardsStatus = 0;
        IdleMode = IDLE_MODE_NONE;
      }
     
      // Playfield X value is only reset during certain modes
      if (PlayfieldMultiplierExpiration) {
        if (CurrentTime > PlayfieldMultiplierExpiration) {
          PlayfieldMultiplierExpiration = 0;
          if (PlayfieldMultiplier > 1) QueueNotification(SOUND_EFFECT_VP_RETURN_TO_1X, 1);
          PlayfieldMultiplier = 1;
        } else {
          for (byte count = 0;count < 4;count++) {
            if (count != CurrentPlayer) OverrideScoreDisplay(count, PlayfieldMultiplier, DISPLAY_OVERRIDE_ANIMATION_BOUNCE);
          }
          DisplaysNeedRefreshing = true;
        }
      } else if (DisplaysNeedRefreshing) {
        DisplaysNeedRefreshing = false;
        ShowPlayerScores(0xFF, false, false);
      }

      if (SpinnerFrenzyEndTime && CurrentTime > SpinnerFrenzyEndTime) {
        SpinnerFrenzyEndTime = 0;
      }

      break;

    case GAME_MODE_EGGS_START:
      if (GameModeStartTime == 0) {
        GameModeStartTime = CurrentTime;
        GameModeEndTime = CurrentTime + 5000;
        byte eggSector = CurrentTime%6;
        QueueNotification(SOUND_EFFECT_VP_SENSORS_PICKING_UP_EGGS, 5);
        QueueNotification(SOUND_EFFECT_VP_ZONE_1 + eggSector, 5);
        HatchlingsProgress = HATCHLING_ZONE_1 << (unsigned long)eggSector;
        GameModeStage = 0;
        StoriesEngaged[CurrentPlayer] = STORY_EGGS;
      }

      if (CurrentTime>GameModeEndTime) {
        SetGameMode(GAME_MODE_EGGS_IN_PROGRESS);
      }
      break;

    case GAME_MODE_EGGS_IN_PROGRESS:
      if (GameModeStartTime == 0) {
        GameModeStartTime = CurrentTime;
        GameModeStage = 0;
        QueueNotification(SOUND_EFFECT_VP_GET_OVER_THERE_BEFORE_HATCH, 5);
        LastTimePromptPlayed = CurrentTime;
        AccumulatedPlayTime = 0;
      }

      if (GameModeStage==0 && CurrentTime>(GameModeStartTime+EGG_DEVELOPMENT_TIME)) {
        QueueNotification(SOUND_EFFECT_VP_HATCHING_IN_PROGRESS, 5);
        GameModeStage = 1;
        SpreadHatchlings();
        PlaySoundEffect(SOUND_EFFECT_EGGS_HATCHING);
        LastTimePromptPlayed = CurrentTime;
      }

      if (!TimersPaused && GameModeStage) {
        AccumulatedPlayTime += (CurrentTime-LastTimeThroughLoop);
        if (AccumulatedPlayTime>HATCHLING_SPREAD_TIME) {
          AccumulatedPlayTime = 0;
          SpreadHatchlings();
          PlaySoundEffect(SOUND_EFFECT_EGGS_HATCHING);
          if (GameModeStage==1) {
            GameModeStage += 1;
            QueueNotification(SOUND_EFFECT_VP_HATCHLINGS_ON_MOVE, 4);
          } else if (GameModeStage==2) {
            GameModeStage += 1;
            QueueNotification(SOUND_EFFECT_VP_HATCHLINGS_3_BLIPS, 5);
          }
        }
      }

      if (HatchlingsProgress==0) {
        if (GameModeStage==0) {
          QueueNotification(SOUND_EFFECT_VP_EGGS_ERADICATED, 6);
        } else {
          QueueNotification(SOUND_EFFECT_VP_LAST_HATCHLING_DONE, 6);
        }
        SetGameMode(GAME_MODE_EGGS_WON);
      }

      if ((HatchlingsProgress&HATCHLING_ALL)==HATCHLING_ALL) {
        SetGameMode(GAME_MODE_EGGS_LOST);
      }
      break;
    case GAME_MODE_EGGS_LOST:
      if (GameModeStartTime == 0) {
        GameModeStartTime = CurrentTime;
        GameModeEndTime = CurrentTime + 5000;
        HatchlingsProgress = 0;
      }
      specialAnimationRunning = true;
      ShowLampAnimation(2, 20, (CurrentTime-GameModeStartTime), 20, false, true);

      if (CurrentTime>GameModeEndTime) {
        SetGameMode(GAME_MODE_BUILD_WEAPONS);
      }
      break;
    case GAME_MODE_EGGS_WON:
      if (GameModeStartTime == 0) {
        GameModeStartTime = CurrentTime;
        GameModeEndTime = CurrentTime + 5000;
        StoriesFinished[CurrentPlayer] |= STORY_EGGS;
        StartScoreAnimation(EGG_ERADICATION_BONUS * TotalScoreMultiplier);
        HatchlingsProgress = 0;
      }

      specialAnimationRunning = true;
      ShowLampAnimation(1, 20, (CurrentTime-GameModeStartTime), 20, false, false);

      if (CurrentTime>GameModeEndTime) {
        SetGameMode(GAME_MODE_BUILD_WEAPONS);
      }
      break;

    case GAME_MODE_PRESENT_BLUE_LOCK:
    case GAME_MODE_PRESENT_GREEN_LOCK:
      if (GameModeStartTime == 0) {
        GameModeStartTime = CurrentTime;
        QueueNotification(SOUND_EFFECT_VP_HOLD_RIGHT_DISABLE_LOCK, 10);
        GameModeEndTime = CurrentTime + 5000;
        SetGeneralIlluminationOn(false);
      }

      if (GameMode==GAME_MODE_PRESENT_BLUE_LOCK) {
        lockFlag = LOCK_STATUS_BLUE_LOCKED;
        readyFlag = LOCK_STATUS_BLUE_READY;
        oppositeLockFlag = LOCK_STATUS_GREEN_LOCKED;
        kickSol = SOL_RIGHT_SAUCER;
      } else {
        lockFlag = LOCK_STATUS_GREEN_LOCKED;
        readyFlag = LOCK_STATUS_GREEN_READY;
        oppositeLockFlag = LOCK_STATUS_BLUE_LOCKED;
        kickSol = SOL_LEFT_SAUCER;
      }

      specialAnimationRunning = true;
      ShowLampAnimation(0, 41, (CurrentTime-GameModeStartTime), 20, false, false);

      for (byte count = 0;count < 4;count++) {
        if (count != CurrentPlayer) OverrideScoreDisplay(count, ((GameModeEndTime - CurrentTime)/1000)+1, DISPLAY_OVERRIDE_ANIMATION_CENTER);
      }

      if (CurrentTime>GameModeEndTime) {
        ShowPlayerScores(0xFF, false, false);
        if (WOS_ReadSingleSwitchState(SW_RIGHT_FLIPPER)) {
          if (GameMode==GAME_MODE_PRESENT_BLUE_LOCK) SetGameMode(GAME_MODE_ADD_LEFT_WEAPON);
          else SetGameMode(GAME_MODE_ADD_RIGHT_WEAPON);
        } else {
          LockABall(lockFlag, readyFlag);

          // if the player has two locks at 100%
          // then they can move onto Battle Multiball
          if (GetNumPlayerLocks()==2) {
            if (GetMaximumWeaponStrength(false)==100 && GetMaximumWeaponStrength(true)==100) {
              StoriesEngaged[CurrentPlayer] = STORY_ATTACK;
            }
          }

          if (StoriesEngaged[CurrentPlayer]&STORY_DEFEND) {
            SetGameMode(GAME_MODE_DEFEND_MULTIBALL_START);
          } else {
            QueueNotification(SOUND_EFFECT_VP_LOCKED, 10);
  
            if (GetNumPlayerLocks()==2) {
              if (DEBUG_MESSAGES) { 
                Serial.write("Two locks = starting battle\n");
              }
              SetGameMode(GAME_MODE_BATTLE_MULTIBALL_START);  
            } else {
              if (NumberOfBallsInPlay) {
                SetGameMode(GAME_MODE_BUILD_WEAPONS);
              } else {
                // See if there's an unearned lock
                if (!KickUnearnedLock(oppositeLockFlag, kickSol)) {
                  // There was no unearned lock, so we have to add a ball
                  if (!AddABall()) {
                    // There are no balls to add, so we can move to multiball
                    // (but this should never happen)
                    SetGameMode(GAME_MODE_BATTLE_MULTIBALL_START); 
                  }
                } else {
                  // We kicked an unearned lock, so we're back to building weapons
                  SetGameMode(GAME_MODE_BUILD_WEAPONS);
                }
              }
            }
          }
        }
      }
      
      break;
    case GAME_MODE_ADD_LEFT_WEAPON:
    case GAME_MODE_ADD_RIGHT_WEAPON:
      if (GameModeStartTime == 0) {
        GameModeStartTime = CurrentTime;
        QueueNotification(SOUND_EFFECT_VP_SELECT_WEAOPNS_RIGHT_FLIPPER, 10);
        GameModeEndTime = CurrentTime + 3000;
        CurrentWeaponSelected = GetNextUnselectedWeapon( (GameMode==GAME_MODE_ADD_LEFT_WEAPON), (GameMode==GAME_MODE_ADD_RIGHT_WEAPON));
      }

      if (GameMode==GAME_MODE_ADD_LEFT_WEAPON) kickSol = SOL_LEFT_SAUCER;
      else kickSol = SOL_RIGHT_SAUCER;
      
      if (WeaponsStatusShownTime) {
        if (CurrentTime>(WeaponsStatusShownTime+5000)) {
          WeaponsStatusShownTime = 0;
          ShowPlayerScores(0xFF, false, false);
        }
      }

      for (byte count = 0;count < 4;count++) {
        if (count != CurrentPlayer) OverrideScoreDisplay(count, ((GameModeEndTime - CurrentTime)/1000)+1, DISPLAY_OVERRIDE_ANIMATION_CENTER);
      }

      if (CurrentTime>GameModeEndTime) {
        ShowPlayerScores(0xFF, false, false);
        SetGameMode(GAME_MODE_BUILD_WEAPONS);
        ReleaseBallFromSaucer(kickSol);
      }
      break;

    case GAME_MODE_BATTLE_MULTIBALL_START:
      if (GameModeStartTime == 0) {
        Audio.StopAllAudio();
        SetGeneralIlluminationOn(false);
        GameModeStartTime = CurrentTime;
        PlaySoundEffect(SOUND_EFFECT_BATTLE_MULTIBALL_START);
        LastTimePromptPlayed = CurrentTime;
        GameModeStage = 0;
        BallSaveEndTime = 0;
      }

      if (BonusLeft[CurrentPlayer]>1 && CurrentTime>(LastTimePromptPlayed+200)) {
        CurrentScores[CurrentPlayer] += 1000 * TotalScoreMultiplier * ((unsigned long)BonusX[CurrentPlayer]);
        BonusLeft[CurrentPlayer] -= 1;
        PlaySoundEffect(SOUND_EFFECT_BONUS_COLLECT);
        LastTimePromptPlayed = CurrentTime;
      } else if (BonusRight[CurrentPlayer]>1 && CurrentTime>(LastTimePromptPlayed+200)) {
        CurrentScores[CurrentPlayer] += 1000 * TotalScoreMultiplier * ((unsigned long)BonusX[CurrentPlayer]);
        BonusRight[CurrentPlayer] -= 1;
        PlaySoundEffect(SOUND_EFFECT_BONUS_COLLECT);
        LastTimePromptPlayed = CurrentTime;
      } else if (GameModeStage==0) {
        GameModeEndTime = CurrentTime + 12000;
        PlaySoundEffect(SOUND_EFFECT_CREATURE_COMING);
        WOS_TurnOffAllLamps();
        GameModeStage = 1;
        LastTimePromptPlayed = CurrentTime;
      } else if (GameModeStage==1) {
        ShowLampAnimation(0, 50, CurrentTime-LastTimePromptPlayed, 20, false, false);
        specialAnimationRunning = true;
        if (CurrentTime>(LastTimePromptPlayed+1234)) {
          WOS_TurnOffAllLamps();
          GameModeStage = 2;
        }
      } else if (GameModeStage==2) {
        ShowLampAnimation(0, 50, CurrentTime-(LastTimePromptPlayed+1234), 2, false, true);
        specialAnimationRunning = true;
        if (CurrentTime>(LastTimePromptPlayed+2511)) {
          WOS_TurnOffAllLamps();
          GameModeStage = 3;
        }
      } else if (GameModeStage==3) {
        ShowLampAnimation(2, 50, CurrentTime-(LastTimePromptPlayed+2511), 22, false, false);
        specialAnimationRunning = true;
        if (CurrentTime>(LastTimePromptPlayed+3796)) {
          WOS_TurnOffAllLamps();
          GameModeStage = 4;
        }
      } else if (GameModeStage==4) {
        ShowLampAnimation(1, 50, CurrentTime-(LastTimePromptPlayed+3796), 2, false, true);
        specialAnimationRunning = true;
        if (CurrentTime>(LastTimePromptPlayed+5429)) {
          WOS_TurnOffAllLamps();
          GameModeStage = 5;
        }
      } else if (GameModeStage==5) {
        ShowLampAnimation(2, 50, CurrentTime-(LastTimePromptPlayed+5429), 2, false, true);
        specialAnimationRunning = true;
        if (CurrentTime>(LastTimePromptPlayed+8000)) {
          WOS_TurnOffAllLamps();
          GameModeStage = 6;
        }
      } else if (GameModeStage==6 && CurrentTime>(LastTimePromptPlayed+9000)) {
        PlaySoundEffect(SOUND_EFFECT_CREATURE_SCREAM_LEFT_1+(CurrentTime%10));
        GameModeStage = 2;
        LastTimePromptPlayed = CurrentTime;
      }

      if (GameModeEndTime!=0 && (CurrentTime>GameModeEndTime || RightFlipperHeld)) {
        ShowPlayerScores(0xFF, false, false);
        SetGameMode(GAME_MODE_BATTLE_MULTIBALL);
      }
      
      break;
    case GAME_MODE_BATTLE_MULTIBALL:
      if (GameModeStartTime == 0) {
        GameModeStartTime = CurrentTime;
        PlayBackgroundSong(SOUND_EFFECT_BATTLE_SONG_1+2);
        SetGeneralIlluminationOn(true);
        WOS_SetLampState(LAMP_HEAD_RIGHT_TIMER, 1, 0, 250);
        ReleaseLock(LOCK_STATUS_BLUE_LOCKED, SW_LEFT_SAUCER, SOL_LEFT_SAUCER);
        LastTimePromptPlayed = CurrentTime;
        GameModeStage = 0;
//        GameModeEndTime = CurrentTime + 100000;
//        IncreasePlayfieldMultiplier(30000);
        CreatureHitsLeft = 1;
        CreatureHitsRight = 1;
        CreatureHitsLeftDecreasing = false;
        CreatureHitsRightDecreasing = false;
        LastCreatureHitLeftTime = 0;
        LastCreatureHitRightTime = 0;
        LeftJackpotTime = 0;
        RightJackpotTime = 0;
        GIFlashIndex = 0;
        GIFlashTimer = CurrentTime;
        QueueNotification(SOUND_EFFECT_VP_BATTLE_INSTRUCTIONS, 9);
        LastSwitchHitTime = CurrentTime;
      }

      if (CurrentTime<(LastSwitchHitTime+4100)) {
        if (CurrentTime > (GIFlashTimer + ((unsigned long)GIFlashChangeState[GIFlashIndex]))) {
          SetGeneralIlluminationOn(GIFlashIndex%2);
          GIFlashIndex += 1;
          if (GIFlashIndex>=NUM_GI_FLASH_SEQUENCE_ENTRIES) {
            GIFlashIndex = 0;
            GIFlashTimer = CurrentTime;
          }
          if (!(GIFlashIndex%2)) PlaySoundEffect(SOUND_EFFECT_BONUS_TIMER_EXPLOSION);
          WOS_SetDisplayCredits(GIFlashIndex);
        }/* else if (GIFlashIndex==(NUM_GI_FLASH_SEQUENCE_ENTRIES-1)) {
          SetGeneralIlluminationOn((CurrentTime/100)%2);
        } */
      } else {
        GIFlashIndex = 0;
        GIFlashTimer = CurrentTime;
        SetGeneralIlluminationOn(true);
      }

      if (!TimersPaused) {
        WOS_SetLampState(LAMP_HEAD_RIGHT_TIMER, 1, 0, 250);
        BonusTimeAccumulated[CurrentPlayer] += (CurrentTime-LastTimeThroughLoop);
        if (BonusTimeAccumulated[CurrentPlayer]>99000) BonusTimeAccumulated[CurrentPlayer] = 99000;
      } else {
        WOS_SetLampState(LAMP_HEAD_RIGHT_TIMER, 0);
      }
      WOS_SetDisplayBallInPlay(BonusTimeAccumulated[CurrentPlayer]/1000, ((CurrentTime/250)%2), true);
      CurrentAchievements[CurrentPlayer] = CurrentBallInPlay;

      if (CurrentTime>(LastTimePromptPlayed+2000) && GameModeStage==0) {
        LastTimePromptPlayed = CurrentTime;
        GameModeStage = 1;
        ReleaseLock(LOCK_STATUS_GREEN_LOCKED, SW_RIGHT_SAUCER, SOL_RIGHT_SAUCER);
        LastTimePromptPlayed = CurrentTime;
      } else if (CurrentTime>(LastTimePromptPlayed+2000) && GameModeStage==1) {
        GameModeStage = 2;
        QueueNotification(SOUND_EFFECT_VP_ENGAGING_THE_CREATURE, 9);
        QueueNotification(SOUND_EFFECT_VP_IN, 9);
        QueueNotification(SOUND_EFFECT_VP_BATTLE, 9);
        LastTimePromptPlayed = CurrentTime;
      }

/*
      for (byte count = 0;count < 4;count++) {
        if (count != CurrentPlayer) OverrideScoreDisplay(count, ((GameModeEndTime - CurrentTime)/1000)+1, DISPLAY_OVERRIDE_ANIMATION_CENTER);
      }
*/      

/*
      if (CreatureHitsLeft>=10 && CreatureHitsRight>=10) {
        SetGameMode(GAME_MODE_BATTLE_FINISHING_SHOT);
        QueueNotification(SOUND_EFFECT_VP_ON_THE_RUN, 7);
      }
*/
      // decrease left hits or right hits if you're at max
      // and haven't hit one in a bit (and you're not paused)
      if (1) {
        if (CreatureHitsLeft==11) {
          if (CurrentTime>(LastCreatureHitLeftTime+10000)) {
            CreatureHitsLeftDecreasing = true;
          }
        }

        if (CreatureHitsRight==11) {
          if (CurrentTime>(LastCreatureHitRightTime+10000)) {
            CreatureHitsRightDecreasing = true;
          }
        }

        if (CreatureHitsLeftDecreasing) {
          byte decreaseAmount = (CurrentTime-LastCreatureHitLeftTime-10000)/2000;
          if (decreaseAmount<11) {
            CreatureHitsLeft = 11 - decreaseAmount;
          } else {
            CreatureHitsLeft = 1;
            CreatureHitsLeftDecreasing = false;
          }
        }

        if (CreatureHitsRightDecreasing) {
          byte decreaseAmount = (CurrentTime-LastCreatureHitRightTime-10000)/2000;
          if (decreaseAmount<11) {
            CreatureHitsRight = 11 - decreaseAmount;
          } else {
            CreatureHitsRight = 1;
            CreatureHitsRightDecreasing = false;
          }
        }
      }

      if (LeftJackpotTime && CurrentTime>(LeftJackpotTime+5000)) {
        LeftJackpotTime = 0;
        CreatureHitsLeft = 1;
      }
      if (RightJackpotTime && CurrentTime>(RightJackpotTime+5000)) {
        RightJackpotTime = 0;
        CreatureHitsRight = 1;
      }

      if (LeftJackpotTime || RightJackpotTime) {
        unsigned long mostRecentJackpot = LeftJackpotTime;
        if (RightJackpotTime>mostRecentJackpot) mostRecentJackpot = RightJackpotTime;
        if (CurrentTime<(mostRecentJackpot+1000)) {
          SetGeneralIlluminationOn((CurrentTime/75)%2?true:false);
          specialAnimationRunning = true;
          ShowLampAnimationSingleStep(2, (CurrentTime/30)%NUM_LAMP_ANIMATIONS);          
        }
      }      

      if (CountBallsInTrough()) {
        if ((StoriesFinished[CurrentPlayer]&STORY_ATTACK)==0 && (StoriesFinished[CurrentPlayer]&STORY_RESTART)==0) {
          SetGameMode(GAME_MODE_WAIT_TO_RESTART_BATTLE);
        } else {
          for (int weapon=0;weapon<8;weapon++) WeaponsStatus[CurrentPlayer][weapon] = 0;
          CurrentWeaponsEngaged[CurrentPlayer] = 0;        
          WOS_SetLampState(LAMP_HEAD_RIGHT_TIMER, 0);
          WOS_SetDisplayBallInPlay(CurrentBallInPlay);
          CurrentAchievements[CurrentPlayer] = 0;
          ShowPlayerScores(0xFF, false, false);
          QueueNotification(SOUND_EFFECT_VP_GOOD_ENOUGH_FOR_NOW, 8);
          QueueNotification(SOUND_EFFECT_VP_WEAPONS_TOASTED, 8);
          PlayBackgroundSong(SOUND_EFFECT_BACKGROUND_SONG_1 + ((CurrentTime / 10) % NUM_BACKGROUND_SONGS));
          SetGameMode(GAME_MODE_BUILD_WEAPONS);
        }
      }
      break;
    case GAME_MODE_WAIT_TO_RESTART_BATTLE:
      if (GameModeStartTime == 0) {
        GameModeStartTime = CurrentTime;
        SetGeneralIlluminationOn(false);
        LastTimePromptPlayed = CurrentTime;
        GameModeStage = 1;
        GameModeEndTime = CurrentTime + 15000;
        QueueNotification(SOUND_EFFECT_VP_HIT_SAUCER_TO_RESTART_BATTLE, 7);
      }

      if (GameModeStage<=1 && CurrentTime>(GameModeEndTime-6000)) {
        GameModeStage = 2;
        QueueNotification(SOUND_EFFECT_VP_FIVE, 5);
      } else if (GameModeStage==2 && CurrentTime>(GameModeEndTime-5000)) {
        GameModeStage = 3;
        QueueNotification(SOUND_EFFECT_VP_FOUR, 5);
      } else if (GameModeStage==3 && CurrentTime>(GameModeEndTime-3000)) {
        GameModeStage = 4;
        QueueNotification(SOUND_EFFECT_VP_THREE, 5);
      } else if (GameModeStage==4 && CurrentTime>(GameModeEndTime-2000)) {
        GameModeStage = 5;
        QueueNotification(SOUND_EFFECT_VP_TWO, 5);
      } else if (GameModeStage==5 && CurrentTime>(GameModeEndTime-1000)) {
        GameModeStage = 6;
        QueueNotification(SOUND_EFFECT_VP_ONE, 5);
      }

      for (byte count = 0;count < 4;count++) {
        if (count != CurrentPlayer) OverrideScoreDisplay(count, ((GameModeEndTime - CurrentTime)/1000)+1, DISPLAY_OVERRIDE_ANIMATION_CENTER);
      }
      
      if (CurrentTime>GameModeEndTime) {
        ShowPlayerScores(0xFF, false, false);
        for (int weapon=0;weapon<8;weapon++) WeaponsStatus[CurrentPlayer][weapon] /= 2;
        CurrentWeaponsEngaged[CurrentPlayer] = 0;        
        WOS_SetLampState(LAMP_HEAD_RIGHT_TIMER, 0);
        WOS_SetDisplayBallInPlay(CurrentBallInPlay);
        CurrentAchievements[CurrentPlayer] = 0;
        ShowPlayerScores(0xFF, false, false);
        QueueNotification(SOUND_EFFECT_VP_BATTLE_LOST_REGROUP, 8);
        QueueNotification(SOUND_EFFECT_VP_WEAPONS_TOASTED, 8);
        PlayBackgroundSong(SOUND_EFFECT_BACKGROUND_SONG_1 + ((CurrentTime / 10) % NUM_BACKGROUND_SONGS));
        SetGameMode(GAME_MODE_BUILD_WEAPONS);
      }
      break;      
    case GAME_MODE_RESTART_BATTLE:
      if (GameModeStartTime == 0) {
        GameModeStartTime = CurrentTime;
        QueueNotification(SOUND_EFFECT_VP_PLUNGE_TO_RESTART, 7);
        StoriesFinished[CurrentPlayer] |= STORY_RESTART;
        AddABall(false);
        ShowPlayerScores(0xFF, false, false);
      }

      if (LastSwitchHitTime>(GameModeStartTime) && CurrentTime>(GameModeStartTime+1000)) {
        // As soon as a switch has been hit and the beginning is over, we can start
        if (!ReleaseLock(LOCK_STATUS_BLUE_LOCKED, SW_LEFT_SAUCER, SOL_LEFT_SAUCER)) {
          ReleaseLock(LOCK_STATUS_GREEN_LOCKED, SW_RIGHT_SAUCER, SOL_RIGHT_SAUCER);
        }
        SetGameMode(GAME_MODE_BATTLE_MULTIBALL);
      }      
      break;
    case GAME_MODE_BATTLE_FINISHING_SHOT:
      if (GameModeStartTime == 0) {
        GameModeStartTime = CurrentTime;
        Audio.StopAllAudio();
        GameModeEndTime = CurrentTime + 45000;
        SetGeneralIlluminationOn(false);
        LastTimePromptPlayed = CurrentTime;
        GameModeStage = 0;
      }
      for (byte count = 0;count < 4;count++) {
        if (count != CurrentPlayer) OverrideScoreDisplay(count, ((GameModeEndTime - CurrentTime)/1000)+1, DISPLAY_OVERRIDE_ANIMATION_CENTER);
      }
      specialAnimationRunning = true;
      ShowLeftSaucerLamps();
      ShowRightSaucerLamps();

      if (GameModeStage==0 && CurrentTime>(LastTimePromptPlayed+10000)) {
        GameModeStage = 1;
        QueueNotification(SOUND_EFFECT_VP_PARK_WEAPON, 7);
      }
          
      if (CountBallsInTrough() || (CurrentTime>GameModeEndTime)) {
        ShowPlayerScores(0xFF, false, false);
        PlayBackgroundSong(SOUND_EFFECT_BACKGROUND_SONG_1 + ((CurrentTime / 10) % NUM_BACKGROUND_SONGS));
        SetGameMode(GAME_MODE_BUILD_WEAPONS);
      }
      break;
    case GAME_MODE_BATTLE_WON:
      if (GameModeStartTime == 0) {
        GameModeStartTime = CurrentTime;
        GameModeEndTime = CurrentTime + 5000;
        SetGeneralIlluminationOn(true);
        PlaySoundEffect(SOUND_EFFECT_BATTLE_WON);
        QueueNotification(SOUND_EFFECT_VP_WON_THE_BATTLE, 9);
        if (StoriesFinished[CurrentPlayer]&STORY_ATTACK) {
          StartScoreAnimation(300000 * PlayfieldMultiplier);
        } else {
          StartScoreAnimation(150000 * PlayfieldMultiplier);
        }
        StoriesFinished[CurrentPlayer] |= STORY_ATTACK;
      } 

      specialAnimationRunning = true;
      ShowLampAnimationSingleStep(3, (CurrentTime/30)%NUM_LAMP_ANIMATIONS);
            
      if (CurrentTime>GameModeEndTime) {
        ReleaseLock(LOCK_STATUS_BLUE_LOCKED, SW_LEFT_SAUCER, SOL_LEFT_SAUCER);
        ReleaseLock(LOCK_STATUS_GREEN_LOCKED, SW_RIGHT_SAUCER, SOL_RIGHT_SAUCER);
        QueueNotification(SOUND_EFFECT_VP_WEAPONS_COOKED, 10);
        SetGameMode(GAME_MODE_BUILD_WEAPONS);
      }
      break;
    case GAME_MODE_BATTLE_LOST:
      if (GameModeStartTime == 0) {
        GameModeStartTime = CurrentTime;
        GameModeEndTime = CurrentTime + 3000;
        SetGeneralIlluminationOn(true);
        PlaySoundEffect(SOUND_EFFECT_BATTLE_LOST);
        QueueNotification(SOUND_EFFECT_VP_BATTLE_LOST_REGROUP, 8);
      }
      if (CurrentTime>GameModeEndTime) {
        ReleaseLock(LOCK_STATUS_BLUE_LOCKED, SW_LEFT_SAUCER, SOL_LEFT_SAUCER);
        ReleaseLock(LOCK_STATUS_GREEN_LOCKED, SW_RIGHT_SAUCER, SOL_RIGHT_SAUCER);
        SetGameMode(GAME_MODE_BUILD_WEAPONS);
      }
      break;
    case GAME_MODE_DEFEND_MULTIBALL_START:
      if (GameModeStartTime == 0) {
        GameModeStartTime = CurrentTime;
        SetGeneralIlluminationOn(false);
        GameModeStage = 0;
        if (StoriesFinished[CurrentPlayer]&STORY_CHASE) {
          QueueNotification(SOUND_EFFECT_VP_DEFEND_SECOND_TRY, 10);
        } else {
          QueueNotification(SOUND_EFFECT_VP_DEFENSE_START, 10);
          QueueNotification(SOUND_EFFECT_VP_CHARGE_OR_LOCK, 10);
        }
        GameModeEndTime = CurrentTime + 6000;
        CreatureDefense = 0;
        DefenseBonus = 1000;
      } 

      if (GameModeStage==0 && (CurrentTime>(GameModeStartTime + 7000) || RightFlipperHeld)) {
        Audio.StopAllNotifications();
        PlaySoundEffect(SOUND_EFFECT_CREATURE_ANGRY_1);
        GameModeStage = 1;
        if (RightFlipperHeld) GameModeEndTime = CurrentTime;
        // Figure out which ball needs to be kicked.
        if (NumberOfBallsLocked==2) {
          // We've stolen a lock, so we're going to kick from that          
          if (!KickUnearnedLock(LOCK_STATUS_GREEN_LOCKED, SOL_RIGHT_SAUCER)) {
            KickUnearnedLock(LOCK_STATUS_BLUE_LOCKED, SOL_LEFT_SAUCER);
          }
        } else {
          // We need to eject a new ball
          AddABall(false);
        }
      }

      if (GameModeStage==1 && (CurrentTime>(GameModeStartTime + 20000))) {
        GameModeStage = 2;
        QueueNotification(SOUND_EFFECT_VP_PLUNGE_SECOND_DEFEND, 10);
      }
      
      if (GameModeEndTime && CurrentTime>GameModeEndTime && LastSwitchHitTime>(GameModeStartTime+1000)) {
        PlayBackgroundSong(SOUND_EFFECT_BATTLE_SONG_1);
        SetGeneralIlluminationOn(true);
        // As soon as a switch has been hit and the beginning is over, we can start
        if (!ReleaseLock(LOCK_STATUS_BLUE_LOCKED, SW_LEFT_SAUCER, SOL_LEFT_SAUCER)) {
          ReleaseLock(LOCK_STATUS_GREEN_LOCKED, SW_RIGHT_SAUCER, SOL_RIGHT_SAUCER);
        }

        LockStatus[CurrentPlayer] |= LOCK_STATUS_BLUE_READY | LOCK_STATUS_GREEN_READY;
        SetGameMode(GAME_MODE_DEFEND_MULTIBALL);
      }
      break;
    case GAME_MODE_DEFEND_MULTIBALL:
      if (GameModeStartTime == 0) {
        GameModeStartTime = CurrentTime;
        SetGeneralIlluminationOn(true);
        GameModeEndTime = 0;
        GameModeStage = 0;
        //QueueNotification(SOUND_EFFECT_VP_DEFEND_MULTIBALL_START, 10);
        DefendLeftPowerUpStart = 0;
        DefendRightPowerUpStart = 0;
      }

      if (!TimersPaused) {
        WOS_SetLampState(LAMP_HEAD_RIGHT_TIMER, 1, 0, 250);
        BonusTimeAccumulated[CurrentPlayer] += (CurrentTime-LastTimeThroughLoop);
        if (BonusTimeAccumulated[CurrentPlayer]>99000) BonusTimeAccumulated[CurrentPlayer] = 99000;
      } else {
        WOS_SetLampState(LAMP_HEAD_RIGHT_TIMER, 0);
      }
      WOS_SetDisplayBallInPlay(BonusTimeAccumulated[CurrentPlayer]/1000, ((CurrentTime/250)%2), true);
      CurrentAchievements[CurrentPlayer] = CurrentBallInPlay;

      // the Defend power-up display will be handled by the ShowLeftSaucerLamps and ShowRightSaucerLamps
      // functions, and if both balls are locked, the Handle Left Saucer (or Right) will do the mode 
      // switch. All we do here is prompts, track damage, and kick the ball if power-up finishes
      if (DefendLeftPowerUpStart && CurrentTime>(DefendLeftPowerUpStart+20000)) {
        DefendLeftPowerUpStart = 0;
        ReleaseLock(LOCK_STATUS_BLUE_LOCKED, SW_LEFT_SAUCER, SOL_LEFT_SAUCER);
        LockStatus[CurrentPlayer] |= LOCK_STATUS_BLUE_READY;
        Audio.StopSound(SOUND_EFFECT_DEFEND_POWERUP);
      }

      if (DefendRightPowerUpStart && CurrentTime>(DefendRightPowerUpStart+20000)) {
        DefendRightPowerUpStart = 0;
        ReleaseLock(LOCK_STATUS_GREEN_LOCKED, SW_RIGHT_SAUCER, SOL_RIGHT_SAUCER);
        LockStatus[CurrentPlayer] |= LOCK_STATUS_GREEN_READY;
        Audio.StopSound(SOUND_EFFECT_DEFEND_POWERUP);
      }
      
      if (CountBallsInTrough() && CurrentTime>(GameModeStartTime+500)) {
        WOS_SetDisplayBallInPlay(CurrentBallInPlay);
        CurrentAchievements[CurrentPlayer] = 0;
        Audio.StopSound(SOUND_EFFECT_DEFEND_POWERUP);
        ShowPlayerScores(0xFF, false, false);
        PlayBackgroundSong(SOUND_EFFECT_BACKGROUND_SONG_1 + ((CurrentTime / 10) % NUM_BACKGROUND_SONGS));
        SetGeneralIlluminationOn(false);
        SetGameMode(GAME_MODE_DEFEND_MULTIBALL_LOST);
      }
      break;
    case GAME_MODE_DEFEND_MULTIBALL_WAIT_FOR_LOCK:
      if (GameModeStartTime == 0) {
        Audio.StopSound(SOUND_EFFECT_DEFEND_POWERUP);
        QueueNotification(SOUND_EFFECT_VP_LOCK_IT_UP, 7);        
        GameModeStartTime = CurrentTime;
        GameModeEndTime = 0;
        GameModeStage = 0;
      }

      // If we've locked a ball, then we won the mode
      if (GetNumPlayerLocks()) {
        SetGameMode(GAME_MODE_DEFEND_MULTIBALL_WON);
      }

//      specialAnimationRunning = true;
//      ShowLampAnimation(5, 10, CurrentTime, 18, false, false);

      for (byte count = 0;count < 4;count++) {
        if (count != CurrentPlayer) OverrideScoreDisplay(count, DefenseBonus, DISPLAY_OVERRIDE_ANIMATION_FLUTTER);
      }

      // If a ball lands in the trough, then we've lost the multiball
      if (CountBallsInTrough()) {
        ShowPlayerScores(0xFF, false, false);
        QueueNotification(SOUND_EFFECT_VP_SCORPION_3, 10);
        SetGeneralIlluminationOn(false);
        SetGameMode(GAME_MODE_DEFEND_MULTIBALL_LOST);
        PlayBackgroundSong(BACKGROUND_TRACK_NONE);
      }
      break;
    case GAME_MODE_DEFEND_MULTIBALL_WON:
      if (GameModeStartTime == 0) {
        WOS_SetDisplayBallInPlay(CurrentBallInPlay);
        CurrentAchievements[CurrentPlayer] = 0;
        Audio.StopSound(SOUND_EFFECT_DEFEND_POWERUP);
        ShowPlayerScores(0xFF, false, false);
        GameModeStartTime = CurrentTime;
        GameModeEndTime = CurrentTime + 5000;
        QueueNotification(SOUND_EFFECT_VP_NICE_WORK_DEFENSES_HELD, 10);
        PlaySoundEffect(SOUND_EFFECT_BATTLE_WON);
        WOS_TurnOffAllLamps();
        ShowShootAgainLamps();
        StoriesFinished[CurrentPlayer] |= STORY_DEFEND;
        StartScoreAnimation(100000 * PlayfieldMultiplier);
      }

      specialAnimationRunning = true;
      ShowLampAnimation(2, 75, CurrentTime, 18, false, false);
      
      if (GameModeEndTime && CurrentTime>GameModeEndTime) {
        if (GetNumPlayerLocks()==2) {
          // we have to eject one of the balls for the next mode
          if (!ReleaseLock(LOCK_STATUS_BLUE_LOCKED, SW_LEFT_SAUCER, SOL_LEFT_SAUCER)) {
            ReleaseLock(LOCK_STATUS_GREEN_LOCKED, SW_RIGHT_SAUCER, SOL_RIGHT_SAUCER);
          }
        }
        SetGeneralIlluminationOn(true);
        SetGameMode(GAME_MODE_CHASE_START);
      }
      break;
    case GAME_MODE_DEFEND_MULTIBALL_LOST:
      if (GameModeStartTime == 0) {
        WOS_SetDisplayBallInPlay(CurrentBallInPlay);
        CurrentAchievements[CurrentPlayer] = 0;
        Audio.StopSound(SOUND_EFFECT_DEFEND_POWERUP);
        GameModeStartTime = CurrentTime;
        GameModeEndTime = CurrentTime + 5000;
        //QueueNotification(SOUND_EFFECT_VP_REBUILD_DEFENSES, 10);
        
        // Check to see if we have to kick a lock in order
        // to return to single ball play
        if (CountBallsInTrough()) {
          if (!ReleaseLock(LOCK_STATUS_BLUE_LOCKED, SW_LEFT_SAUCER, SOL_LEFT_SAUCER)) {
            ReleaseLock(LOCK_STATUS_GREEN_LOCKED, SW_RIGHT_SAUCER, SOL_RIGHT_SAUCER);
          }
        }
        // Disable locks (make them requalify)
        LockStatus[CurrentPlayer] &= ~(LOCK_STATUS_BLUE_READY | LOCK_STATUS_GREEN_READY);
        GameModeStage = 0;
        if (DefendRightPowerUpStart || DefendLeftPowerUpStart) {
          QueueNotification(SOUND_EFFECT_VP_DEFEND_LOST_DURING_POWERUP, 9);
        } else if (CreatureDefense==100) {
          QueueNotification(SOUND_EFFECT_VP_DEFEND_LOST_100, 9);
        } else {
          QueueNotification(SOUND_EFFECT_VP_FAILED_TO_DEFEND_CREATURE_MOVING, 9);
        }
      }
      if (GameModeStage==0 && CurrentTime>(GameModeStartTime+1000)) {
        SetGeneralIlluminationOn(true);
        GameModeStage = 1;
      }

      specialAnimationRunning = true;
      ShowLampAnimation(1, 100, CurrentTime, 5, false, true);

      // Playfield X value is only reset during certain modes
      if (PlayfieldMultiplierExpiration) {
        if (CurrentTime > PlayfieldMultiplierExpiration) {
          PlayfieldMultiplierExpiration = 0;
          if (PlayfieldMultiplier > 1) QueueNotification(SOUND_EFFECT_VP_RETURN_TO_1X, 1);
          PlayfieldMultiplier = 1;
        } else {
          for (byte count = 0;count < 4;count++) {
            if (count != CurrentPlayer) OverrideScoreDisplay(count, PlayfieldMultiplier, DISPLAY_OVERRIDE_ANIMATION_BOUNCE);
          }
          DisplaysNeedRefreshing = true;
        }
      } else if (DisplaysNeedRefreshing) {
        DisplaysNeedRefreshing = false;
        ShowPlayerScores(0xFF, false, false);
      }

      if (GameModeEndTime && CurrentTime>GameModeEndTime) {
//        PlayBackgroundSong(SOUND_EFFECT_BACKGROUND_SONG_1 + ((CurrentTime / 10) % NUM_BACKGROUND_SONGS));
        SetGameMode(GAME_MODE_CHASE_START);
      }
      break;

    case GAME_MODE_CHASE_START:
      if (GameModeStartTime == 0 ) {
        ChaseLegBonus = 0;
        GameModeStartTime = CurrentTime;
        GameModeEndTime = CurrentTime + 5000;
        GameModeStage = 0;
        StoriesEngaged[CurrentPlayer] = STORY_CHASE;
        SetGeneralIlluminationOn(false);
/*        
        if (StoriesFinished[CurrentPlayer]&STORY_CHASE) {
          QueueNotification(SOUND_EFFECT_VP_CHASE_IT_QUICK, 10);
        } else {
          QueueNotification(SOUND_EFFECT_VP_CHASE_IT_GENERIC, 10);
          QueueNotification(SOUND_EFFECT_VP_SENSORS_GET_ON_IT, 10);
        }
*/
        PlayBackgroundSong(SOUND_EFFECT_BATTLE_SONG_1+4);
        QueueNotification(SOUND_EFFECT_VP_CREATURE_FLEEING, 9);
        QueueNotification(SOUND_EFFECT_VP_SPINNER_STARTS_CHASE_1, 9);
        ChasePromptPlayed = CurrentTime;     
      }

      if (GameModeStage==0 && CurrentTime>(GameModeStartTime+1000)) {
        GameModeStage = 1;
        SetGeneralIlluminationOn(true);
      }

      if (CurrentTime>GameModeEndTime) {
        SetGameMode(GAME_MODE_CHASE_SPINNER);
      }
      break;

    case GAME_MODE_CHASE_SPINNER:
      if (GameModeStartTime == 0 ) {
        GameModeStartTime = CurrentTime;
        if (StoriesFinished[CurrentPlayer]&STORY_CHASE) {
          GameModeEndTime = CurrentTime + 20000;
        } else {
          GameModeEndTime = CurrentTime + 40000;
        }
        GameModeStage = 0;
        LastTimePromptPlayed = 0;
        ChaseLegBonus = 30000;
        DisplaysNeedRefreshing = false;
      }

      for (byte count = 0;count < 4;count++) {
        if (count != CurrentPlayer) OverrideScoreDisplay(count, ((GameModeEndTime - CurrentTime)/1000)+1, DISPLAY_OVERRIDE_ANIMATION_CENTER);
      }
      
      if (CurrentTime>(GameModeEndTime)) {
        ShowPlayerScores(0xFF, false, false);
        QueueNotification(SOUND_EFFECT_VP_CREATURE_OUT_OF_RANGE, 8);
        QueueNotification(SOUND_EFFECT_VP_NO_CHASE_TODAY, 8);
        PlayBackgroundSong(SOUND_EFFECT_BACKGROUND_SONG_1 + ((CurrentTime / 10) % NUM_BACKGROUND_SONGS));
        SetGameMode(GAME_MODE_BUILD_WEAPONS);
      }
      break;
      
    case GAME_MODE_CHASE_TOP_LANES:
    case GAME_MODE_CHASE_POP_BUMPERS:
    case GAME_MODE_CHASE_STANDUPS:
    case GAME_MODE_CHASE_LANE_TO_TOP:
    case GAME_MODE_CHASE_6_DROPS:
    case GAME_MODE_CHASE_5_DROPS:
    case GAME_MODE_CHASE_BULLSEYE:
      if (GameModeStartTime == 0 ) {
        GameModeStartTime = CurrentTime;
        if (StoriesFinished[CurrentPlayer]&STORY_CHASE) {
          GameModeEndTime = CurrentTime + 20000;
        } else {
          GameModeEndTime = 0;
        }
        GameModeStage = 0;
        SetGeneralIlluminationOn(false);
        LastTimePromptPlayed = 0;
        ChaseLegBonus = 30000;
        ShowPlayerScores(0xFF, false, false);
        DisplaysNeedRefreshing = false;
      }

      if (CurrentTime<(GameModeStartTime+30000)) {
        ChaseLegBonus = (30000 - (CurrentTime-GameModeStartTime));
        ChaseLegBonus = ((ChaseLegBonus/100)*100);
        if (ChaseLegBonus<10000) ChaseLegBonus = 10000;
      } else {
        ChaseLegBonus = 10000;
      }

      if (ChaseLegBonus>10000) {
        OverrideScoreDisplay(CurrentPlayer, ChaseLegBonus, DISPLAY_OVERRIDE_ANIMATION_FLUTTER);
        DisplaysNeedRefreshing = true;
      } else if (DisplaysNeedRefreshing) {
        // Playfield X value is only reset during certain modes
        if (PlayfieldMultiplierExpiration) {
          if (CurrentTime > PlayfieldMultiplierExpiration) {
            PlayfieldMultiplierExpiration = 0;
            if (PlayfieldMultiplier > 1) QueueNotification(SOUND_EFFECT_VP_RETURN_TO_1X, 1);
            PlayfieldMultiplier = 1;
          } else if (!GameModeEndTime) {
            for (byte count = 0;count < 4;count++) {
              if (count != CurrentPlayer) OverrideScoreDisplay(count, PlayfieldMultiplier, DISPLAY_OVERRIDE_ANIMATION_BOUNCE);
            }
            DisplaysNeedRefreshing = true;
          }
        } else if (DisplaysNeedRefreshing) {
          DisplaysNeedRefreshing = false;
          ShowPlayerScores(0xFF, false, false);
        }
      }

      if (RightFlipperHeld || CurrentTime<(GameModeStartTime+1000)) {
        SetGeneralIlluminationOn(false);
        unsigned long animationTime = (CurrentTime - RightFlipperDownTime);
        if (CurrentTime<(GameModeStartTime+1000)) animationTime = (CurrentTime-GameModeStartTime);

        if (LastTimePromptPlayed==0) {
          PlaySoundEffect(SOUND_EFFECT_SCANNER_TICK);
          LastTimePromptPlayed = animationTime;
        } else if (animationTime>(LastTimePromptPlayed+1000)) {
          PlaySoundEffect(SOUND_EFFECT_SCANNER_TICK);
          LastTimePromptPlayed = animationTime;
        }
        
        if (GameMode==GAME_MODE_CHASE_BULLSEYE && CurrentTime<(GameModeStartTime+1000)) {
          specialAnimationRunning = true;
          ShowLampAnimation(4, 20, animationTime, LAMP_ANIMATION_STEPS-5, false, false);
        } else {
          specialAnimationRunning = true;
          ShowChaseScannerLamps(animationTime%1000, GameMode);
        }        
      } else {
        LastTimePromptPlayed = 0;
        SetGeneralIlluminationOn(true);
        if (GameModeStage==0) {
          GameModeStage += 1;
          if (GameMode==GAME_MODE_CHASE_BULLSEYE) {
            QueueNotification(SOUND_EFFECT_VP_CHASE_BULLSEYES, 10);        
          } else if (ChasePromptPlayed==0 || CurrentTime>(ChasePromptPlayed+20000)) {
            QueueNotification(SOUND_EFFECT_VP_CHASE_THE_THINGS, 10);
            ChasePromptPlayed = CurrentTime;
          }
        } else if (GameModeStage==1 && CurrentTime>(ChasePromptPlayed+10000)) {
          ChasePromptPlayed = CurrentTime;
          if (GameMode!=GAME_MODE_CHASE_BULLSEYE) QueueNotification(SOUND_EFFECT_VP_RIGHT_FLIPPER_SCAN_CHASE, 10);
          GameModeStage = 2;
        }
      }

      if (GameModeEndTime) {
        for (byte count = 0;count < 4;count++) {
          if (count != CurrentPlayer) OverrideScoreDisplay(count, ((GameModeEndTime - CurrentTime)/1000)+1, DISPLAY_OVERRIDE_ANIMATION_CENTER);
        }
      }

      if (GameModeEndTime && CurrentTime>GameModeEndTime) {
        ShowPlayerScores(0xFF, false, false);
        SetGameMode(GAME_MODE_CHASE_LOST);      
      }
      break;

    case GAME_MODE_CHASE_FINAL_SHOT:
      if (GameModeStartTime == 0 ) {
        GameModeStartTime = CurrentTime;
        GameModeEndTime = CurrentTime + 45000;
        WOS_TurnOffAllLamps();
        SetGeneralIlluminationOn(false);
        QueueNotification(SOUND_EFFECT_VP_ALMOST_DONE_CHASE_LOCK_IT, 8);
        ShowPlayerScores(0xFF, false, false);
        GameModeStage = 0;
      }

      if (CurrentTime>(GameModeStartTime+1000)) {
        if (GameModeStage==0) {
          SetGeneralIlluminationOn(true);
          GameModeStage = 1;
        }
      }
            
      if (CurrentTime>(GameModeStartTime+3000)) {
        if (GameModeStage==1) {
          QueueNotification(SOUND_EFFECT_VP_LIMITED_TIME_TO_FINISH_CHASE, 8);
          GameModeStage = 2;
        }
      }
      specialAnimationRunning = true;
      if (LockStatus[CurrentPlayer] & LOCK_STATUS_GREEN_LOCKED) ShowLeftSaucerLamps();
      else ShowRightSaucerLamps();

      for (byte count = 0;count < 4;count++) {
        if (count != CurrentPlayer) OverrideScoreDisplay(count, ((GameModeEndTime - CurrentTime)/1000)+1, DISPLAY_OVERRIDE_ANIMATION_CENTER);
      }

      if (CurrentTime>GameModeEndTime) {
        ShowPlayerScores(0xFF, false, false);
        SetGameMode(GAME_MODE_CHASE_LOST);  
      }
      break;

    case GAME_MODE_CHASE_WON:
      if (GameModeStartTime == 0 ) {
        GameModeStartTime = CurrentTime;
        ShowPlayerScores(0xFF, false, false);
        StoriesFinished[CurrentPlayer] |= STORY_CHASE;
        GameModeEndTime = CurrentTime + 7500;       
        SetGeneralIlluminationOn(true);
        StartScoreAnimation(100000 * PlayfieldMultiplier);
        QueueNotification(SOUND_EFFECT_VP_BREATHING_ROOM_BUILD, 9);
      }

      specialAnimationRunning = true;
      ShowLampAnimation(0, 20, CurrentTime, LAMP_ANIMATION_STEPS-2, false, false);

      if (CurrentTime>GameModeEndTime) {
        ReleaseLock(LOCK_STATUS_BLUE_LOCKED, SW_LEFT_SAUCER, SOL_LEFT_SAUCER);
        ReleaseLock(LOCK_STATUS_GREEN_LOCKED, SW_RIGHT_SAUCER, SOL_RIGHT_SAUCER);
        PlayBackgroundSong(SOUND_EFFECT_BACKGROUND_SONG_1 + ((CurrentTime / 10) % NUM_BACKGROUND_SONGS));
        SetGameMode(GAME_MODE_BUILD_WEAPONS);
      }
      break;
      
    case GAME_MODE_CHASE_LOST:
      if (GameModeStartTime == 0 ) {
        GameModeStartTime = CurrentTime;
        StoriesFinished[CurrentPlayer] |= STORY_CHASE;
        GameModeEndTime = CurrentTime + 5000; 
        QueueNotification(SOUND_EFFECT_VP_CHASE_FAILED, 6);      
        ShowPlayerScores(0xFF, false, false);
        SetGeneralIlluminationOn(true);
      }

      specialAnimationRunning = true;
      ShowLampAnimation(0, 20, CurrentTime, 5, false, true);
    
      if (CurrentTime>GameModeEndTime) {
        PlayBackgroundSong(SOUND_EFFECT_BACKGROUND_SONG_1 + ((CurrentTime / 10) % NUM_BACKGROUND_SONGS));
        SetGameMode(GAME_MODE_BUILD_WEAPONS);
      }
      break;
    case GAME_MODE_VICTORY_START:
      if (GameModeStartTime == 0 ) {
        GameModeStartTime = CurrentTime;
        GameModeEndTime = CurrentTime + 3000;
        BallSaveEndTime = CurrentTime + 3000 + 30000 + 3000 + 1000;
        QueueNotification(SOUND_EFFECT_VP_VICTORY_START, 10);
        SetGeneralIlluminationOn(true);
        StoriesFinished[CurrentPlayer] &= (~ALL_BASIC_STORIES);
      }

      specialAnimationRunning = true;
      ShowLampAnimationSingleStep(2, (CurrentTime/30)%NUM_LAMP_ANIMATIONS);

      if (CurrentTime>GameModeEndTime) {
        SetGameMode(GAME_MODE_VICTORY);
      }
      break;
    case GAME_MODE_VICTORY:
      if (GameModeStartTime == 0 ) {
        GameModeStartTime = CurrentTime;
        GameModeEndTime = CurrentTime + 60000;
        SetGeneralIlluminationOn(true);
        QueueNotification(SOUND_EFFECT_VP_VICTORY_INSTRUCTIONS, 10);
        DropTargets6.ResetDropTargets(CurrentTime + 200, true);
        DropTargets5.ResetDropTargets(CurrentTime + 500, true);
        if (StoriesFinished[CurrentPlayer]&STORY_VICTORY) {
          IncreasePlayfieldMultiplier(60000);
        }
        LeftVictoryReady = false;
        RightVictoryReady = false;
        LeftJackpotTime = 0;
        RightJackpotTime = 0;
      }

      if (CurrentTime<(GameModeStartTime+2000)) {
        specialAnimationRunning = true;
        ShowLampAnimationSingleStep(1, (CurrentTime/30)%NUM_LAMP_ANIMATIONS);
      }

      if (LeftJackpotTime || RightJackpotTime) {
        StoriesFinished[CurrentPlayer] |= STORY_VICTORY;
        if (CurrentTime>(LeftJackpotTime+2000)) LeftJackpotTime = 0;
        if (CurrentTime>(RightJackpotTime+2000)) RightJackpotTime = 0;
        SetGeneralIlluminationOn((CurrentTime/75)%2?true:false);
        specialAnimationRunning = true;
        ShowLampAnimationSingleStep(2, (CurrentTime/30)%NUM_LAMP_ANIMATIONS);
      } else {
        SetGeneralIlluminationOn(true);
      }

      for (byte count = 0;count < 4;count++) {
        if (count != CurrentPlayer) OverrideScoreDisplay(count, ((GameModeEndTime - CurrentTime)/1000)+1, DISPLAY_OVERRIDE_ANIMATION_CENTER);
      }

      if (CurrentTime>GameModeEndTime) {
        ShowPlayerScores(0xFF, false, false);
        SetGameMode(GAME_MODE_VICTORY_END);        
      }
      break;

    case GAME_MODE_VICTORY_END:
      if (GameModeStartTime == 0 ) {
        GameModeStartTime = CurrentTime;
        GameModeEndTime = CurrentTime + 3000;
        if (BallSaveEndTime) BallSaveEndTime = GameModeEndTime;
        SetGeneralIlluminationOn(true);
        if (StoriesFinished[CurrentPlayer]&STORY_VICTORY) QueueNotification(SOUND_EFFECT_VP_VICTORY_DONE_EVERYTHING, 9);
        else QueueNotification(SOUND_EFFECT_VP_VICTORY_END_NO_JACKPOT, 9);
      }

      specialAnimationRunning = true;
      ShowLampAnimationSingleStep(0, (CurrentTime/30)%NUM_LAMP_ANIMATIONS);

      if (CurrentTime>GameModeEndTime) {
        SetGameMode(GAME_MODE_BUILD_WEAPONS);
      }
      break;      
  }

  if ( !specialAnimationRunning && NumTiltWarnings <= MaxTiltWarnings ) {
    ShowTopLaneLamps();
    ShowLeftSaucerLamps();
    ShowRightSaucerLamps();
    ShowBonusXLamps();
    ShowStandupLamps();
    Show3DropLamps();
    ShowTimerLamps();
    ShowShootAgainLamps();
    ShowPopBumperLamps();
  }


  // Three types of display modes are shown here:
  // 1) score animation
  // 2) fly-bys
  // 3) normal scores
  if (ScoreAdditionAnimationStartTime != 0) {
    // Score animation
    if ((CurrentTime - ScoreAdditionAnimationStartTime) < 2000) {
      byte displayPhase = (CurrentTime - ScoreAdditionAnimationStartTime) / 60;
      byte digitsToShow = 1 + displayPhase / 6;
      if (digitsToShow > 6) digitsToShow = 6;
      unsigned long scoreToShow = ScoreAdditionAnimation;
      for (byte count = 0;count < (6 - digitsToShow);count++) {
        scoreToShow = scoreToShow / 10;
      }
      if (scoreToShow == 0 || displayPhase % 2) scoreToShow = DISPLAY_OVERRIDE_BLANK_SCORE;
      byte countdownDisplay = (1 + CurrentPlayer) % 4;

      for (byte count = 0;count < 4;count++) {
        if (count == countdownDisplay) OverrideScoreDisplay(count, scoreToShow, DISPLAY_OVERRIDE_ANIMATION_NONE);
        else if (count != CurrentPlayer) OverrideScoreDisplay(count, DISPLAY_OVERRIDE_BLANK_SCORE, DISPLAY_OVERRIDE_ANIMATION_NONE);
      }
    } else {
      byte countdownDisplay = (1 + CurrentPlayer) % 4;
      unsigned long remainingScore = 0;
      if ( (CurrentTime - ScoreAdditionAnimationStartTime) < 5000 ) {
        remainingScore = (((CurrentTime - ScoreAdditionAnimationStartTime) - 2000) * ScoreAdditionAnimation) / 3000;
        if ((remainingScore / 1000) != (LastRemainingAnimatedScoreShown / 1000)) {
          LastRemainingAnimatedScoreShown = remainingScore;
          PlaySoundEffect(SOUND_EFFECT_SCORE_TICK);
        }
      } else {
        CurrentScores[CurrentPlayer] += ScoreAdditionAnimation;
        remainingScore = 0;
        ScoreAdditionAnimationStartTime = 0;
        ScoreAdditionAnimation = 0;
      }

      for (byte count = 0;count < 4;count++) {
        if (count == countdownDisplay) OverrideScoreDisplay(count, ScoreAdditionAnimation - remainingScore, DISPLAY_OVERRIDE_ANIMATION_NONE);
        else if (count != CurrentPlayer) OverrideScoreDisplay(count, DISPLAY_OVERRIDE_BLANK_SCORE, DISPLAY_OVERRIDE_ANIMATION_NONE);
        else OverrideScoreDisplay(count, CurrentScores[CurrentPlayer] + remainingScore, DISPLAY_OVERRIDE_ANIMATION_NONE);
      }
    }
    if (ScoreAdditionAnimationStartTime) ShowPlayerScores(CurrentPlayer, false, false);
    else ShowPlayerScores(0xFF, false, false);
/*  } else if (LastSpinnerHit != 0 && TotalSpins[CurrentPlayer]<SpinnerMaxGoal) {
    OverrideScoreDisplay(CurrentPlayer, SpinnerMaxGoal-TotalSpins[CurrentPlayer], DISPLAY_OVERRIDE_ANIMATION_NONE);
    if (CurrentTime>(LastSpinnerHit+3000)) {
      LastSpinnerHit = 0;
      ShowPlayerScores(0xFF, false, false);
    } else {
      ShowPlayerScores(CurrentPlayer, false, false);
    } */
  } else {
    ShowPlayerScores(CurrentPlayer, (BallFirstSwitchHitTime == 0) ? true : false, (BallFirstSwitchHitTime > 0 && ((CurrentTime - LastTimeScoreChanged) > 2000)) ? true : false);

    // Show the player up lamp
    if (BallFirstSwitchHitTime == 0) {
      for (byte count = 0;count < 4;count++) {
        WOS_SetLampState(LAMP_HEAD_PLAYER_1_UP + count, (((CurrentTime / 250) % 2) == 0 || CurrentPlayer != count) ? false : true);
        WOS_SetLampState(LAMP_HEAD_1_PLAYER + count, ((count+1)==CurrentNumPlayers) ? true : false);
      }
    } else {
      for (byte count = 0;count < 4;count++) {
        WOS_SetLampState(LAMP_HEAD_PLAYER_1_UP + count, (CurrentPlayer == count) ? true : false);
        WOS_SetLampState(LAMP_HEAD_1_PLAYER + count, ((count+1)==CurrentNumPlayers) ? true : false);
      }
    }
  }

  // Check to see if ball is in the trough
  if (CountBallsInTrough()>(TotalBallsLoaded-(NumberOfBallsInPlay+NumberOfBallsLocked))) {

    if (BallTimeInTrough == 0) {
      // If this is the first time we're seeing too many balls in the trough, we'll wait to make sure 
      // everything is settled
      BallTimeInTrough = CurrentTime;
    } else {
      
      // Make sure the ball stays on the sensor for at least
      // 0.5 seconds to be sure that it's not bouncing or passing through
      if ((CurrentTime - BallTimeInTrough) > 750) {

        if (BallFirstSwitchHitTime == 0 && NumTiltWarnings <= MaxTiltWarnings) {
          // Nothing hit yet, so return the ball to the player
          WOS_PushToTimedSolenoidStack(SOL_BALL_RAMP_THROWER, 16, CurrentTime);
          BallTimeInTrough = 0;
          returnState = MACHINE_STATE_NORMAL_GAMEPLAY;
        } else {
          // if we haven't used the ball save, and we're under the time limit, then save the ball
          if (BallSaveEndTime && CurrentTime<(BallSaveEndTime+BALL_SAVE_GRACE_PERIOD)) {
            WOS_PushToTimedSolenoidStack(SOL_BALL_RAMP_THROWER, 16, CurrentTime + 100);
            QueueNotification(SOUND_EFFECT_VP_SHOOT_AGAIN, 10);
            
            WOS_SetLampState(LAMP_SHOOT_AGAIN, 0);
            BallTimeInTrough = CurrentTime;
            returnState = MACHINE_STATE_NORMAL_GAMEPLAY;

            // Only 1 ball save if one ball in play
            if (NumberOfBallsInPlay==1) {
              BallSaveEndTime = CurrentTime + 1000;
            } else {
              if (CurrentTime>BallSaveEndTime) BallSaveEndTime += 1000;
            }
           
          } else {

            NumberOfBallsInPlay -= 1;
            if (NumberOfBallsInPlay==0) {
              ShowPlayerScores(0xFF, false, false);
              Audio.StopAllAudio();
              StoriesEngaged[CurrentPlayer] = 0;
              returnState = MACHINE_STATE_COUNTDOWN_BONUS;
            }
          }
        }
      }
    }
  } else {
    BallTimeInTrough = 0;
  }

  if (OutholeCheckTime==0 || CurrentTime>(OutholeCheckTime+2000)) {
    OutholeCheckTime = CurrentTime;
    MoveBallFromOutholeToRamp(false);
  }

  LastTimeThroughLoop = CurrentTime;
  return returnState;
}



unsigned long CountdownStartTime = 0;
unsigned long LastCountdownReportTime = 0;
unsigned long BonusCountDownEndTime = 0;
byte DecrementingBonusCounter;
byte IncrementingBonusXCounter;
byte TotalLeftBonus = 0;
byte TotalRightBonus = 0;
byte TotalBonusX = 0;
byte TotalTimerBonus = 0;
byte BonusRunoffPhase = 0;
boolean CountdownBonusHurryUp = false;

int CountDownDelayTimes[] = {100, 80, 70, 60, 50, 40, 30, 20, 20, 20};

int CountdownBonus(boolean curStateChanged) {

  // If this is the first time through the countdown loop
  if (curStateChanged) {

    CountdownStartTime = CurrentTime;
    LastCountdownReportTime = CurrentTime;
    ShowBonusXLamps();
    ShowLeftSaucerLamps();
    ShowRightSaucerLamps();
    IncrementingBonusXCounter = 1;
    DecrementingBonusCounter = BonusLeft[CurrentPlayer];
    TotalLeftBonus = BonusLeft[CurrentPlayer];
    TotalRightBonus = BonusRight[CurrentPlayer];
    TotalBonusX = BonusX[CurrentPlayer];
    BonusRunoffPhase = 0;
    CountdownBonusHurryUp = false;
    TotalTimerBonus = (byte)(BonusTimeAccumulated[CurrentPlayer] / 1000);

    BonusCountDownEndTime = 0xFFFFFFFF;
  }

//  unsigned long countdownDelayTime = 180 - IncrementingBonusXCounter*15;
  unsigned long countdownDelayTime = (unsigned long)(CountDownDelayTimes[IncrementingBonusXCounter-1]);
  if (CountdownBonusHurryUp && countdownDelayTime>((unsigned long)CountDownDelayTimes[9])) countdownDelayTime=CountDownDelayTimes[9];

  if (TotalTimerBonus) {
    WOS_SetDisplayBallInPlay(TotalTimerBonus, ((CurrentTime/250)%2), true);    
    CurrentAchievements[CurrentPlayer] = CurrentBallInPlay;
  }
  
  if ((CurrentTime - LastCountdownReportTime) > countdownDelayTime) {

    if (TotalTimerBonus) {
      WOS_SetLampState(LAMP_HEAD_RIGHT_TIMER, 1, 0, 250);
      WOS_SetLampState(LAMP_COLLECT_DOUBLE_TIMER, DoubleTimerBonus[CurrentPlayer], 0, 125);
      // Only give sound & score if this isn't a tilt
      if (NumTiltWarnings <= MaxTiltWarnings) {
        PlaySoundEffect(SOUND_EFFECT_BONUS_TIMER_PAYOUT);
        CurrentScores[CurrentPlayer] += 1000;
      }

      TotalTimerBonus -= 1;
      if (TotalTimerBonus==0) {
        WOS_SetLampState(LAMP_COLLECT_DOUBLE_TIMER, 0);
        WOS_SetLampState(LAMP_HEAD_RIGHT_TIMER, 0);
        WOS_SetDisplayBallInPlay(CurrentBallInPlay);
        CurrentAchievements[CurrentPlayer] = 0;
      }
    } else if (DecrementingBonusCounter) {
      
      // Only give sound & score if this isn't a tilt
      if (NumTiltWarnings <= MaxTiltWarnings) {
        PlaySoundEffect(SOUND_EFFECT_BONUS_COUNT);
        CurrentScores[CurrentPlayer] += 1000;
      }

      DecrementingBonusCounter -= 1;
      if (BonusRunoffPhase==0) {
        BonusLeft[CurrentPlayer] = DecrementingBonusCounter;
        ShowLeftSaucerLamps();
      } else {
        BonusRight[CurrentPlayer] = DecrementingBonusCounter;
        ShowRightSaucerLamps();
      }

    } else if (BonusCountDownEndTime==0xFFFFFFFF) {
      if (BonusRunoffPhase==0) {
        BonusRunoffPhase = 1;
        DecrementingBonusCounter = TotalRightBonus;
      } else {
        IncrementingBonusXCounter += 1;
        if (BonusX[CurrentPlayer]>1) {
          DecrementingBonusCounter = TotalLeftBonus;
          BonusLeft[CurrentPlayer] = TotalLeftBonus;
          BonusRight[CurrentPlayer] = TotalRightBonus;
          ShowLeftSaucerLamps();
          ShowRightSaucerLamps();
          BonusX[CurrentPlayer] -= 1;
          BonusRunoffPhase = 0;
          if (BonusX[CurrentPlayer]==9) BonusX[CurrentPlayer] = 8;
        } else {
          BonusX[CurrentPlayer] = TotalBonusX;
          BonusLeft[CurrentPlayer] = TotalLeftBonus;
          BonusRight[CurrentPlayer] = TotalRightBonus;
          BonusCountDownEndTime = CurrentTime + 1000;
        }
      }
    }
    LastCountdownReportTime = CurrentTime;
  }

  if (CurrentTime > BonusCountDownEndTime) {

    // Reset any lights & variables of goals that weren't completed
    BonusCountDownEndTime = 0xFFFFFFFF;
    return MACHINE_STATE_BALL_OVER;
  }

  return MACHINE_STATE_COUNTDOWN_BONUS;
}



void CheckHighScores() {
  unsigned long highestScore = 0;
  int highScorePlayerNum = 0;
  for (int count = 0;count < CurrentNumPlayers;count++) {
    if (CurrentScores[count] > highestScore) highestScore = CurrentScores[count];
    highScorePlayerNum = count;
  }

  if (highestScore > HighScore) {
    HighScore = highestScore;
    if (HighScoreReplay) {
      AddCredit(false, 3);
      WOS_WriteULToEEProm(WOS_TOTAL_REPLAYS_EEPROM_START_BYTE, WOS_ReadULFromEEProm(WOS_TOTAL_REPLAYS_EEPROM_START_BYTE) + 3);
    }
    WOS_WriteULToEEProm(WOS_HIGHSCORE_EEPROM_START_BYTE, highestScore);
    WOS_WriteULToEEProm(WOS_TOTAL_HISCORE_BEATEN_START_BYTE, WOS_ReadULFromEEProm(WOS_TOTAL_HISCORE_BEATEN_START_BYTE) + 1);

    for (int count = 0;count < 4;count++) {
      if (count == highScorePlayerNum) {
        WOS_SetDisplay(count, CurrentScores[count], true, 2);
      } else {
        WOS_SetDisplayBlank(count, 0x00);
      }
    }

    WOS_PushToTimedSolenoidStack(SOL_KNOCKER, 8, CurrentTime, true);
    WOS_PushToTimedSolenoidStack(SOL_KNOCKER, 8, CurrentTime + 300, true);
    WOS_PushToTimedSolenoidStack(SOL_KNOCKER, 8, CurrentTime + 600, true);
  }
}


unsigned long MatchSequenceStartTime = 0;
unsigned long MatchDelay = 150;
byte MatchDigit = 0;
byte NumMatchSpins = 0;
byte ScoreMatches = 0;

int ShowMatchSequence(boolean curStateChanged) {
  if (!MatchFeature) return MACHINE_STATE_ATTRACT;

  if (curStateChanged) {
    MatchSequenceStartTime = CurrentTime;
    MatchDelay = 1500;
    MatchDigit = CurrentTime % 10;
    NumMatchSpins = 0;
    WOS_SetLampState(LAMP_HEAD_MATCH, 1, 0);
    WOS_SetDisableFlippers();
    ScoreMatches = 0;
  }

  if (NumMatchSpins < 40) {
    if (CurrentTime > (MatchSequenceStartTime + MatchDelay)) {
      MatchDigit += 1;
      if (MatchDigit > 9) MatchDigit = 0;
      //PlaySoundEffect(10+(MatchDigit%2));
      PlaySoundEffect(SOUND_EFFECT_MATCH_SPIN);
      WOS_SetDisplayBallInPlay((int)MatchDigit * 10);
      MatchDelay += 50 + 4 * NumMatchSpins;
      NumMatchSpins += 1;
      WOS_SetLampState(LAMP_HEAD_MATCH, NumMatchSpins % 2, 0);

      if (NumMatchSpins == 40) {
        WOS_SetLampState(LAMP_HEAD_MATCH, 0);
        MatchDelay = CurrentTime - MatchSequenceStartTime;
      }
    }
  }

  if (NumMatchSpins >= 40 && NumMatchSpins <= 43) {
    if (CurrentTime > (MatchSequenceStartTime + MatchDelay)) {
      if ( (CurrentNumPlayers > (NumMatchSpins - 40)) && ((CurrentScores[NumMatchSpins - 40] / 10) % 10) == MatchDigit) {
        ScoreMatches |= (1 << (NumMatchSpins - 40));
        AddSpecialCredit();
        MatchDelay += 1000;
        NumMatchSpins += 1;
        WOS_SetLampState(LAMP_HEAD_MATCH, 1);
      } else {
        NumMatchSpins += 1;
      }
      if (NumMatchSpins == 44) {
        MatchDelay += 5000;
      }
    }
  }

  if (NumMatchSpins > 43) {
    if (CurrentTime > (MatchSequenceStartTime + MatchDelay)) {
      return MACHINE_STATE_ATTRACT;
    }
  }

  for (int count = 0;count < 4;count++) {
    if ((ScoreMatches >> count) & 0x01) {
      // If this score matches, we're going to flash the last two digits
      if ( (CurrentTime / 200) % 2 ) {
        WOS_SetDisplayBlank(count, WOS_GetDisplayBlank(count) & 0x0F);
      } else {
        WOS_SetDisplayBlank(count, WOS_GetDisplayBlank(count) | 0x30);
      }
    }
  }

  return MACHINE_STATE_MATCH_MODE;
}




////////////////////////////////////////////////////////////////////////////
//
//  Switch Handling functions
//
////////////////////////////////////////////////////////////////////////////
boolean RegisterWeaponsHit(byte weaponsIndex, byte weaponsBitmask, byte weaponsAdvance) {
  if (weaponsIndex>8) return false;
  boolean progressAdded = false;

  if (GameMode==GAME_MODE_BATTLE_MULTIBALL) {
    if ((weaponsIndex%2)==0) {
      PlaySoundEffect(SOUND_EFFECT_CREATURE_SCREAM_RIGHT_1 + (CreatureHitsRight%5));
      if (CreatureHitsRight==10) QueueNotification(SOUND_EFFECT_VP_BATTLE_RIGHT_SIDE_READY, 7);
      CreatureHitsRight += 1;
      CreatureHitsRightDecreasing = false;
      LastCreatureHitRightTime = CurrentTime;
      if (CreatureHitsRight>11) CreatureHitsRight = 11;
    } else {
      PlaySoundEffect(SOUND_EFFECT_CREATURE_SCREAM_LEFT_1 + (CreatureHitsLeft%5));
      if (CreatureHitsLeft==10) QueueNotification(SOUND_EFFECT_VP_BATTLE_LEFT_SIDE_READY, 7);
      CreatureHitsLeft += 1;
      CreatureHitsLeftDecreasing = false;
      LastCreatureHitLeftTime = CurrentTime;
      if (CreatureHitsLeft>11) CreatureHitsLeft = 11;
    }
    CurrentScores[CurrentPlayer] += 1000 * TotalScoreMultiplier;

    if (CurrentTime>(GameModeEndTime-20000)) {
      GameModeEndTime += 7500;
    }
  } else if (GameMode==GAME_MODE_BUILD_WEAPONS) {

    boolean announcementDone = false;
    if (StoriesEngaged[CurrentPlayer] & STORY_DEFEND || StoriesEngaged[CurrentPlayer] & STORY_ATTACK) {

      if (CurrentWeaponsEngaged[CurrentPlayer] & weaponsBitmask) {

        if (DEBUG_MESSAGES) {
            if (weaponsAdvance==100) {
//            char buf[64];
//            sprintf(buf, "Hit %d (mask=0x%02X) for %d\n", weaponsIndex, weaponsBitmask, weaponsAdvance);
//            Serial.write(buf);
          }
        }
        
        BuildWeaponsLastModeHit = CurrentTime;        
        byte statusBefore = WeaponsStatus[CurrentPlayer][weaponsIndex-1];
        WeaponsStatus[CurrentPlayer][weaponsIndex-1] += (weaponsAdvance * NumberOfBallsInPlay);
       
        if (statusBefore==0 && (StoriesEngaged[CurrentPlayer]&STORY_DEFEND)) {
          if (weaponsIndex%2) QueueNotification(SOUND_EFFECT_VP_LEFT_WEAPON_ACTIVATED, 5);
          else QueueNotification(SOUND_EFFECT_VP_RIGHT_WEAPON_ACTIVATED, 5);
        }

        if (WeaponsStatus[CurrentPlayer][weaponsIndex-1]>=100) {
          WeaponsStatus[CurrentPlayer][weaponsIndex-1] = 100;
          if (statusBefore<100) announcementDone = CheckForFoundLock(weaponsIndex);
        } else {
          WeaponsProgressChangedTime = CurrentTime;
          if (weaponsIndex%2) LeftWeaponChangedTime = CurrentTime;
          else RightWeaponChangedTime = CurrentTime;
          LastWeaponChanged = weaponsIndex;
        }
        // Set game lock based on weapon 
        SetQualifiedGameLocks(weaponsIndex);

        if (!announcementDone) AnnounceWeaponsProgress(statusBefore, weaponsIndex);
        if (statusBefore!=WeaponsStatus[CurrentPlayer][weaponsIndex-1]) {
          progressAdded = true;
        }
      }
      
    }
  } else if (GameMode==GAME_MODE_DEFEND_MULTIBALL) {
    CreatureDefense += WeaponsStatus[CurrentPlayer][weaponsIndex-1];
    if (CreatureDefense>=100) {
      CreatureDefense = 100;
      SetGameMode(GAME_MODE_DEFEND_MULTIBALL_WAIT_FOR_LOCK);
    }
    progressAdded = true;
    // Should play special sound here
  } else if (GameMode==GAME_MODE_DEFEND_MULTIBALL_WAIT_FOR_LOCK) {
    DefenseBonus += (((unsigned long)10) * ((unsigned long)WeaponsStatus[CurrentPlayer][weaponsIndex-1]));
    if (DefenseBonus>100000) DefenseBonus = 100000;
  }

  return progressAdded;
}

void HandleStandupSwitches(byte switchHit) {
  byte switchIndex = switchHit - SW_1_TARGET;

  HatchlingsProgress &= ~(HATCHLING_ZONE_1<<switchIndex);

  if (GameMode==GAME_MODE_PRE_PLUNGE_CHOICE || GameMode==GAME_MODE_SKILL_SHOT) {
    PlaySoundEffect(SOUND_EFFECT_SKILL_SHOT);
    StartScoreAnimation(((unsigned long)CurrentBallInPlay) * SUPER_SKILL_SHOT_AWARD);
  } else {

    byte oldStandupStatus = StandupStatus[CurrentPlayer];
    StandupStatus[CurrentPlayer] |= 0x01<<switchIndex;
    boolean newTargetHit = false;

    if (StandupStatus[CurrentPlayer]==0x07) {
      StandupsCompletedTime = CurrentTime;
      StandupLevel[CurrentPlayer] += 1;
      if (StandupLevel[CurrentPlayer]==0 && TopLaneLevel[CurrentPlayer]>=1) {
        QueueNotification(SOUND_EFFECT_VP_LOWER_POP_HOLDOVER, 7);
      } else if (StandupLevel[CurrentPlayer]==1 && TopLaneLevel[CurrentPlayer]>=2) {
        QueueNotification(SOUND_EFFECT_VP_RIGHT_POP_HOLDOVER, 7);
      } else if (StandupLevel[CurrentPlayer]>=2 && TopLaneLevel[CurrentPlayer]>=3) {
        QueueNotification(SOUND_EFFECT_VP_UPPER_POP_HOLDOVER, 7);
      }
      StandupStatus[CurrentPlayer] = 0;
      StartScoreAnimation(((unsigned long)StandupLevel[CurrentPlayer])*TotalScoreMultiplier*3000);
      PlaySoundEffect(SOUND_EFFECT_STANDUPS_COMPLETE);
      IncreasePlayfieldMultiplier(20000);
      newTargetHit = true;
    } else {
      AddToBonus((switchIndex<2)?1:0, (switchIndex>0)?1:0);
      if (oldStandupStatus!=StandupStatus[CurrentPlayer]) {
        CurrentScores[CurrentPlayer] += 1000 * TotalScoreMultiplier;
        newTargetHit = true;
        PlaySoundEffect(SOUND_EFFECT_STANDUP_HIT);
      } else {
        CurrentScores[CurrentPlayer] += 100 * TotalScoreMultiplier;
        PlaySoundEffect(SOUND_EFFECT_STANDUP_DUPLICATE);
      }
    }

    if (newTargetHit) RegisterWeaponsHit(WEAPON_INDEX_HARPOONS, WEAPONS_MODE_HARPOONS, WEAPON_ADVANCE_HARPOONS);
  }

}

void HandleTopLaneSwitches(byte switchHit) {
  byte switchIndex = switchHit - SW_4_ROLLOVER;

  HatchlingsProgress &= ~(HATCHLING_ZONE_4<<switchIndex);

  if (switchIndex>2) return;
  // Debounce top lanes
  if (CurrentTime<(LastTopLaneHit[switchIndex]+TOP_LANE_DEBOUNCE_TIME)) return;
  LastTopLaneHit[switchIndex] = CurrentTime;

  if (GameMode==GAME_MODE_PRE_PLUNGE_CHOICE || GameMode==GAME_MODE_SKILL_SHOT) {
    if (SkillShotLane==(0x01<<switchIndex)) {
      PlaySoundEffect(SOUND_EFFECT_SKILL_SHOT);
      StartScoreAnimation(((unsigned long)CurrentBallInPlay)*SKILL_SHOT_AWARD);
      SkillShotLaneHitTime = CurrentTime;
      SkillShotLaneHit = switchIndex;
    } else {
      PlaySoundEffect(SOUND_EFFECT_TOP_LANE_NEW);
      CurrentScores[CurrentPlayer] += 1000;
      AddToBonus(switchIndex<2, switchIndex>0);
    }
    SetGameMode(GAME_MODE_BUILD_WEAPONS);
  } else {
    byte newTopLaneStatus = TopLaneStatus[CurrentPlayer] | (0x01<<switchIndex);
    boolean newRolloverHit = false;
    
    if (newTopLaneStatus==0x07) {
      PlaySoundEffect(SOUND_EFFECT_TOP_LANE_LEVEL_FINISHED);
      TopLaneStatus[CurrentPlayer] = 0;
      if (StandupLevel[CurrentPlayer]>=1 && TopLaneLevel[CurrentPlayer]<1) {
        QueueNotification(SOUND_EFFECT_VP_LOWER_POP_HOLDOVER, 7);
      } else if (StandupLevel[CurrentPlayer]>=2 && TopLaneLevel[CurrentPlayer]==1) {
        QueueNotification(SOUND_EFFECT_VP_RIGHT_POP_HOLDOVER, 7);
      } else if (StandupLevel[CurrentPlayer]>=3 && TopLaneLevel[CurrentPlayer]==2) {
        QueueNotification(SOUND_EFFECT_VP_UPPER_POP_HOLDOVER, 7);
      }
      TopLaneLevel[CurrentPlayer] += 1;
      CurrentScores[CurrentPlayer] += TotalScoreMultiplier * 3000;
      TopLanesCompletedTime = CurrentTime;
      newRolloverHit = true;
      IncreasePlayfieldMultiplier(20000);
    } else {
      if (newTopLaneStatus != TopLaneStatus[CurrentPlayer]) {
        PlaySoundEffect(SOUND_EFFECT_TOP_LANE_NEW);
        CurrentScores[CurrentPlayer] += TotalScoreMultiplier * 1000;
        newRolloverHit = true;
      } else {
        PlaySoundEffect(SOUND_EFFECT_TOP_LANE_REPEAT);
        CurrentScores[CurrentPlayer] += TotalScoreMultiplier * 1000;
      }
      TopLaneStatus[CurrentPlayer] = newTopLaneStatus;
    }

    if (newRolloverHit) RegisterWeaponsHit(WEAPON_INDEX_AIR_ASSAULT, WEAPONS_MODE_AIR_ASSAULT, WEAPON_ADVANCE_AIR_ASSAULT);
  }
}


void CheckForBattleJackpot() {
  if (CreatureHitsLeft<11 && CreatureHitsRight<11) return;

  if (CreatureHitsLeft==11 && CreatureHitsRight==11 && LeftSaucerLastState && RightSaucerLastState) {
    QueueNotification(SOUND_EFFECT_VP_SUPER_JACKPOT_ALT_1, 8);
    StartScoreAnimation(SUPER_JACKPOT_AWARD * PlayfieldMultiplier);
    LeftJackpotTime = 0;
    RightJackpotTime = 0;
    CreatureHitsLeft = 1;
    CreatureHitsRight = 1;
  } else if (CreatureHitsLeft==11 && LeftSaucerLastState && LeftJackpotTime==0) {
    QueueNotification(SOUND_EFFECT_VP_JACKPOT_ALT_1, 8);
    StartScoreAnimation(JACKPOT_AWARD * PlayfieldMultiplier);
    LeftJackpotTime = CurrentTime;
    StoriesFinished[CurrentPlayer] |= STORY_ATTACK;
  } else if (CreatureHitsRight==11 && RightSaucerLastState && RightJackpotTime==0) {
    QueueNotification(SOUND_EFFECT_VP_JACKPOT_ALT_1, 8);
    StartScoreAnimation(JACKPOT_AWARD * PlayfieldMultiplier);
    RightJackpotTime = CurrentTime;    
    StoriesFinished[CurrentPlayer] |= STORY_ATTACK;
  }
}


unsigned long LastTimeLeftSaucerHit = 0;

boolean HandleLeftSaucer() {

  if (LastTimeLeftSaucerHit==0) {
    LastTimeLeftSaucerHit = CurrentTime;
  } else {
    // Debounce to half a second
    if ( (CurrentTime-LastTimeLeftSaucerHit)<500 ) return false;
  }
  LastTimeLeftSaucerHit = CurrentTime;

  // Ignore this switch if a ball is already locked there
  if (CheckLock(LOCK_STATUS_BLUE_LOCKED, SW_LEFT_SAUCER)) {
    return false;
  }
  
  if (GameMode==GAME_MODE_BUILD_WEAPONS || GameMode==GAME_MODE_PRE_PLUNGE_CHOICE || GameMode==GAME_MODE_SKILL_SHOT) {
    RegisterWeaponsHit(WEAPON_INDEX_BLUE_LASER, WEAPONS_MODE_BLUE_LASER, WEAPON_ADVANCE_BLUE_LASER);

    // Check to see if this is a bounce on a locked ball
    if (MachineLocks & LOCK_STATUS_BLUE_LOCKED) {
      // this happens too often on vibration
    } else {     
      // Figure out if we should start a mode or present a weapons selection      
      if (LockStatus[CurrentPlayer] & LOCK_STATUS_BLUE_READY) {
        SetGameMode(GAME_MODE_PRESENT_BLUE_LOCK);
      } else if (!(LockStatus[CurrentPlayer]&LOCK_STATUS_BLUE_LOCKED)) {
        SetGameMode(GAME_MODE_ADD_LEFT_WEAPON);
      }
    }
  } else if (GameMode==GAME_MODE_WAIT_TO_RESTART_BATTLE) {
    SetGameMode(GAME_MODE_RESTART_BATTLE);
  } else if (GameMode==GAME_MODE_DEFEND_MULTIBALL) {
    if (DefendRightPowerUpStart && CurrentTime<(DefendRightPowerUpStart+7000)) {
      QueueNotification(SOUND_EFFECT_VP_CHARGER_OVERLOAD, 4);
      WOS_PushToTimedSolenoidStack(SOL_LEFT_SAUCER, 16, CurrentTime + 100, true);
      DefendRightPowerUpStart = CurrentTime;
      Audio.StopSound(SOUND_EFFECT_DEFEND_POWERUP);
      PlaySoundEffect(SOUND_EFFECT_DEFEND_POWERUP);      
    } else {
      LockABall(LOCK_STATUS_BLUE_LOCKED, LOCK_STATUS_BLUE_READY);
      // Start or complete weapons power up jackpot
      if (GetNumPlayerLocks()==1) {
        // Start a power-up sequence
        DefendLeftPowerUpStart = CurrentTime;
        PlaySoundEffect(SOUND_EFFECT_DEFEND_POWERUP);
        QueueNotification(SOUND_EFFECT_VP_CHARGE_FIRST_BALL, 4);
      } else {
        SetGameMode(GAME_MODE_DEFEND_MULTIBALL_WON);
      }
    }
  } else if (GameMode==GAME_MODE_DEFEND_MULTIBALL_WAIT_FOR_LOCK) {
    LockABall(LOCK_STATUS_BLUE_LOCKED, LOCK_STATUS_BLUE_READY);
  } else if (GameMode==GAME_MODE_BATTLE_MULTIBALL) {
    RegisterWeaponsHit(WEAPON_INDEX_BLUE_LASER, WEAPONS_MODE_BLUE_LASER, 0);
    CheckForBattleJackpot();
    WOS_PushToTimedSolenoidStack(SOL_LEFT_SAUCER, 16, CurrentTime + 5000, true);
  } else if (GameMode==GAME_MODE_BATTLE_FINISHING_SHOT) {
    unsigned long numberOfWeapons100 = 0;
    for (byte count=0;count<8;count++) {
      if (WeaponsStatus[CurrentPlayer][count]==100) numberOfWeapons100 += 1;
    }
    StartScoreAnimation(TotalScoreMultiplier * numberOfWeapons100 * 25000);
    SetGameMode(GAME_MODE_BATTLE_WON);
  } else if (GameMode==GAME_MODE_CHASE_FINAL_SHOT) {
    SetGameMode(GAME_MODE_CHASE_WON);
  } else if (GameMode==GAME_MODE_DEFEND_MULTIBALL_LOST) {
    CurrentScores[CurrentPlayer] += PlayfieldMultiplier * 1000;
    PlaySoundEffect(SOUND_EFFECT_SMALL_EXPLOSION);
    WOS_PushToTimedSolenoidStack(SOL_LEFT_SAUCER, 16, CurrentTime + 1000, true);
  } else if (GameMode>=GAME_MODE_CHASE_START && GameMode<=GAME_MODE_CHASE_LOST) {
    CurrentScores[CurrentPlayer] += PlayfieldMultiplier * 1000;
    PlaySoundEffect(SOUND_EFFECT_SMALL_EXPLOSION);
    WOS_PushToTimedSolenoidStack(SOL_LEFT_SAUCER, 16, CurrentTime + 1000, true);
  } else if (GameMode>=GAME_MODE_EGGS_START && GameMode<=GAME_MODE_EGGS_LOST) {
    CurrentScores[CurrentPlayer] += PlayfieldMultiplier * 1000;
    PlaySoundEffect(SOUND_EFFECT_SMALL_EXPLOSION);
    WOS_PushToTimedSolenoidStack(SOL_LEFT_SAUCER, 16, CurrentTime + 1000, true);
  } else if (GameMode==GAME_MODE_VICTORY && LeftVictoryReady) {
    LeftVictoryReady = false;
    LeftJackpotTime = CurrentTime;
    PlaySoundEffect(SOUND_EFFECT_SMALL_EXPLOSION);
    QueueNotification(SOUND_EFFECT_VP_JACKPOT_ALT_1, 9);
    StartScoreAnimation(VICTORY_JACKPOT_VALUE * TotalScoreMultiplier);
    WOS_PushToTimedSolenoidStack(SOL_LEFT_SAUCER, 16, CurrentTime + 2000, true);
  } else {
    WOS_PushToTimedSolenoidStack(SOL_LEFT_SAUCER, 16, CurrentTime + 100, true);
  }

  return true;
}

unsigned long LastTimeRightSaucerHit = 0;

boolean HandleRightSaucer() {

  if (LastTimeRightSaucerHit==0) {
    LastTimeRightSaucerHit = CurrentTime;
  } else {
    // Debounce to half a second
    if ( (CurrentTime-LastTimeRightSaucerHit)<500 ) return false;
  }  
  LastTimeRightSaucerHit = CurrentTime;

  // Ignore this switch if a ball is already locked there
  if (CheckLock(LOCK_STATUS_GREEN_LOCKED, SW_RIGHT_SAUCER)) {
    return false;
  }
  
  if (GameMode==GAME_MODE_BUILD_WEAPONS || GameMode==GAME_MODE_PRE_PLUNGE_CHOICE || GameMode==GAME_MODE_SKILL_SHOT) {
    RegisterWeaponsHit(WEAPON_INDEX_GREEN_LASER, WEAPONS_MODE_GREEN_LASER, WEAPON_ADVANCE_GREEN_LASER);

    // Check to see if this is a bounce on a locked ball
    if (MachineLocks & LOCK_STATUS_GREEN_LOCKED) {
      // this happens too often on vibration
    } else {     
      // Figure out if we should start a mode or present a weapons selection      
      if (LockStatus[CurrentPlayer] & LOCK_STATUS_GREEN_READY) {
        SetGameMode(GAME_MODE_PRESENT_GREEN_LOCK);
      } else if (!(LockStatus[CurrentPlayer]&LOCK_STATUS_GREEN_LOCKED)) {
        SetGameMode(GAME_MODE_ADD_RIGHT_WEAPON);
      }
    }
  } else if (GameMode==GAME_MODE_WAIT_TO_RESTART_BATTLE) {
    SetGameMode(GAME_MODE_RESTART_BATTLE);
  } else if (GameMode==GAME_MODE_DEFEND_MULTIBALL) {
    if (DefendLeftPowerUpStart && CurrentTime<(DefendLeftPowerUpStart+7000)) {
      QueueNotification(SOUND_EFFECT_VP_CHARGER_OVERLOAD, 4);
      WOS_PushToTimedSolenoidStack(SOL_RIGHT_SAUCER, 16, CurrentTime + 100, true);
      DefendLeftPowerUpStart = CurrentTime;
      Audio.StopSound(SOUND_EFFECT_DEFEND_POWERUP);
      PlaySoundEffect(SOUND_EFFECT_DEFEND_POWERUP);      
    } else {  
      LockABall(LOCK_STATUS_GREEN_LOCKED, LOCK_STATUS_GREEN_READY);
      // Start or complete weapons power up jackpot
      if (GetNumPlayerLocks()==1) {
        // Start a power-up sequence
        DefendRightPowerUpStart = CurrentTime;
        PlaySoundEffect(SOUND_EFFECT_DEFEND_POWERUP);
        QueueNotification(SOUND_EFFECT_VP_CHARGE_FIRST_BALL, 4);
      } else {
        SetGameMode(GAME_MODE_DEFEND_MULTIBALL_WON);
      }
    }
  } else if (GameMode==GAME_MODE_DEFEND_MULTIBALL_WAIT_FOR_LOCK) {
    LockABall(LOCK_STATUS_GREEN_LOCKED, LOCK_STATUS_GREEN_READY);
  } else if (GameMode==GAME_MODE_BATTLE_MULTIBALL) {
    RegisterWeaponsHit(WEAPON_INDEX_GREEN_LASER, WEAPONS_MODE_GREEN_LASER, 0);
    CheckForBattleJackpot();
    WOS_PushToTimedSolenoidStack(SOL_RIGHT_SAUCER, 16, CurrentTime + 5000, true);
  } else if (GameMode==GAME_MODE_BATTLE_FINISHING_SHOT) {
    unsigned long numberOfWeapons100 = 0;
    for (byte count=0;count<8;count++) {
      if (WeaponsStatus[CurrentPlayer][count]==100) numberOfWeapons100 += 1;
    }
    StartScoreAnimation(PlayfieldMultiplier * numberOfWeapons100 * 25000);
    SetGameMode(GAME_MODE_BATTLE_WON);
  } else if (GameMode==GAME_MODE_CHASE_FINAL_SHOT) {
    SetGameMode(GAME_MODE_CHASE_WON);
  } else if (GameMode==GAME_MODE_DEFEND_MULTIBALL_LOST) {
    CurrentScores[CurrentPlayer] += TotalScoreMultiplier * 1000;
    PlaySoundEffect(SOUND_EFFECT_SMALL_EXPLOSION);
    WOS_PushToTimedSolenoidStack(SOL_RIGHT_SAUCER, 16, CurrentTime + 1000, true);
  } else if (GameMode>=GAME_MODE_CHASE_START && GameMode<=GAME_MODE_CHASE_LOST) {
    CurrentScores[CurrentPlayer] += TotalScoreMultiplier * 1000;
    PlaySoundEffect(SOUND_EFFECT_SMALL_EXPLOSION);
    WOS_PushToTimedSolenoidStack(SOL_RIGHT_SAUCER, 16, CurrentTime + 1000, true);    
  } else if (GameMode>=GAME_MODE_EGGS_START && GameMode<=GAME_MODE_EGGS_LOST) {
    CurrentScores[CurrentPlayer] += TotalScoreMultiplier * 1000;
    PlaySoundEffect(SOUND_EFFECT_SMALL_EXPLOSION);
    WOS_PushToTimedSolenoidStack(SOL_RIGHT_SAUCER, 16, CurrentTime + 1000, true);
  } else if (GameMode==GAME_MODE_VICTORY && RightVictoryReady) {
    RightVictoryReady = false;
    RightJackpotTime = CurrentTime;
    PlaySoundEffect(SOUND_EFFECT_SMALL_EXPLOSION);
    QueueNotification(SOUND_EFFECT_VP_JACKPOT_ALT_1, 9);
    StartScoreAnimation(VICTORY_JACKPOT_VALUE * TotalScoreMultiplier);
    WOS_PushToTimedSolenoidStack(SOL_RIGHT_SAUCER, 16, CurrentTime + 2000, true);
  } else {
    WOS_PushToTimedSolenoidStack(SOL_RIGHT_SAUCER, 16, CurrentTime + 100, true);
  }

  return true;
}


void UpdateSaucerStatus(int curState) {
  boolean leftSaucerDownState = false;
  boolean leftSaucerUpState = false;

  if (WOS_ReadSingleSwitchState(SW_LEFT_SAUCER)) {
    LeftSaucerUpTime = 0;
    if (LeftSaucerDownTime==0) {
      LeftSaucerDownTime = CurrentTime;
    } else if (CurrentTime > (LeftSaucerDownTime + 250)) {
      leftSaucerDownState = true;
    }
  } else {
    LeftSaucerDownTime = 0;
    if (LeftSaucerUpTime==0) {
      LeftSaucerUpTime = CurrentTime;
    } else if (CurrentTime > (LeftSaucerUpTime + 250)) {
      leftSaucerUpState = true;
    }
  }

  if (leftSaucerUpState && LeftSaucerLastState) {
    LeftSaucerLastState = false;
  } else if (leftSaucerDownState && !LeftSaucerLastState) {
    LeftSaucerLastState = true;
    if (curState==MACHINE_STATE_NORMAL_GAMEPLAY) {
      if (HandleLeftSaucer()) {
//        if (GameMode==GAME_MODE_CHASE_FINAL_SHOT) SetGameMode(GAME_MODE_CHASE_WON);
        LastSwitchHitTime = CurrentTime;
        LastSwitchHit = SW_LEFT_SAUCER;
        if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
      }
    }
  }

  boolean rightSaucerDownState = false;
  boolean rightSaucerUpState = false;

  if (WOS_ReadSingleSwitchState(SW_RIGHT_SAUCER)) {
    RightSaucerUpTime = 0;
    if (RightSaucerDownTime==0) {
      RightSaucerDownTime = CurrentTime;
    } else if (CurrentTime > (RightSaucerDownTime + 250)) {
      rightSaucerDownState = true;
    }
  } else {
    RightSaucerDownTime = 0;
    if (RightSaucerUpTime==0) {
      RightSaucerUpTime = CurrentTime;
    } else if (CurrentTime > (RightSaucerUpTime + 250)) {
      rightSaucerUpState = true;
    }
  }

  if (rightSaucerUpState && RightSaucerLastState) {
    RightSaucerLastState = false;
  } else if (rightSaucerDownState && !RightSaucerLastState) {
    RightSaucerLastState = true;
    if (curState==MACHINE_STATE_NORMAL_GAMEPLAY) {
      if (HandleRightSaucer()) {
//        if (GameMode==GAME_MODE_CHASE_FINAL_SHOT) SetGameMode(GAME_MODE_CHASE_WON);
        LastSwitchHitTime = CurrentTime;
        LastSwitchHit = SW_RIGHT_SAUCER;
        if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
      }
    }
  }  
}


int HandleSystemSwitches(int curState, byte switchHit) {
  int returnState = curState;
  switch (switchHit) {
    case SW_SELF_TEST_SWITCH:
      returnState = MACHINE_STATE_TEST_BOOT;
      SetLastSelfTestChangedTime(CurrentTime);
      break;
    case SW_COIN_1:
    case SW_COIN_2:
    case SW_COIN_3:
      AddCoinToAudit(SwitchToChuteNum(switchHit));
      AddCoin(SwitchToChuteNum(switchHit));
      break;
    case SW_CREDIT_RESET:
      if (MachineState == MACHINE_STATE_MATCH_MODE) {
        // If the first ball is over, pressing start again resets the game
        if (Credits >= 1 || FreePlayMode) {
          if (!FreePlayMode) {
            Credits -= 1;
            WOS_WriteByteToEEProm(WOS_CREDITS_EEPROM_BYTE, Credits);
            WOS_SetDisplayCredits(Credits, !FreePlayMode);
          }
          returnState = MACHINE_STATE_INIT_GAMEPLAY;
        }
      } else {
        CreditResetPressStarted = CurrentTime;
      }
      break;
    case SW_OUTHOLE:
      MoveBallFromOutholeToRamp(true);
      break;
    case SW_PLUMB_TILT:
    case SW_ROLL_TILT:
    case SW_PLAYFIELD_TILT:
      // This should be debounced
      if ((CurrentTime - LastTiltWarningTime) > TILT_WARNING_DEBOUNCE_TIME) {
        LastTiltWarningTime = CurrentTime;
        NumTiltWarnings += 1;
        if (NumTiltWarnings > MaxTiltWarnings) {
          WOS_DisableSolenoidStack();
          WOS_SetDisableFlippers(true);
          WOS_TurnOffAllLamps();
          WOS_SetLampState(LAMP_HEAD_TILT, 1);
          Audio.StopAllAudio();
        }
        PlaySoundEffect(SOUND_EFFECT_TILT_WARNING);
        OverrideGeneralIllumination(false, CurrentTime + 2000);
      }
      break;
  }



  return returnState;
}

unsigned long Drop6ClearRewards[] = {10000, 20000, 30000, 50000};

boolean Handle6Drops(byte switchHit) {

  byte result;
  unsigned long numTargetsDown = 0;
  
  result = DropTargets6.HandleDropTargetHit(switchHit);
  numTargetsDown = (unsigned long)CountBits(result);

  if (numTargetsDown) CurrentScores[CurrentPlayer] += TotalScoreMultiplier * numTargetsDown * 100;

  byte cleared = DropTargets6.CheckIfBankCleared();

  if (cleared) {
    // currently no award for clearing in order
    DropTargets6.ResetDropTargets(CurrentTime + 500, true);
    if (Drop6Clears[CurrentPlayer]<4) {
      CurrentScores[CurrentPlayer] += Drop6ClearRewards[Drop6Clears[CurrentPlayer]] * TotalScoreMultiplier;
      Drop6Clears[CurrentPlayer] += 1;
    } else {
      AwardSpecial();
      Drop6Clears[CurrentPlayer] = 0;
    }
    IncreasePlayfieldMultiplier(30000);
    PlaySoundEffect(SOUND_EFFECT_DROP_TARGET_RESET);

    if (GameMode==GAME_MODE_VICTORY) LeftVictoryReady = true;
  } else if (numTargetsDown) {
    PlaySoundEffect(SOUND_EFFECT_DROP_TARGET_HIT);
  }

  if (numTargetsDown) {
    RegisterWeaponsHit(WEAPON_INDEX_DEPTH_CHARGES, WEAPONS_MODE_DEPTH_CHARGES, (WEAPON_ADVANCE_DEPTH_CHARGES*numTargetsDown));
  }

  return numTargetsDown?true:false;
}


boolean Handle5Drops(byte switchHit) {

  byte result;
  unsigned long numTargetsDown = 0;
  
  result = DropTargets5.HandleDropTargetHit(switchHit);
  numTargetsDown = (unsigned long)CountBits(result);

  if (numTargetsDown) CurrentScores[CurrentPlayer] += TotalScoreMultiplier * numTargetsDown * 100;

  byte cleared = DropTargets5.CheckIfBankCleared();

  if (cleared) {
    // currently no award for clearing in order
    DropTargets5.ResetDropTargets(CurrentTime + 500, true);
    Drop5Clears[CurrentPlayer] += 1;
    IncreaseBonusX();
    IncreasePlayfieldMultiplier(30000);
    PlaySoundEffect(SOUND_EFFECT_DROP_TARGET_RESET);
    if (NumberOfBallsInPlay==2) {
      DoubleTimerBonus[CurrentPlayer] = true;
    }
    if (GameMode==GAME_MODE_VICTORY) RightVictoryReady = true;
  } else if (numTargetsDown) {
    PlaySoundEffect(SOUND_EFFECT_DROP_TARGET_HIT);
  }

  if (numTargetsDown) {
    RegisterWeaponsHit(WEAPON_INDEX_TORPEDOES, WEAPONS_MODE_TORPEDOES, (WEAPON_ADVANCE_TORPEDOES*numTargetsDown));
  }

  return numTargetsDown?true:false;
}


void AdvanceChase() {
  unsigned long chaseMultiplier = 1;

  if (GameMode==GAME_MODE_CHASE_SPINNER) {
    QueueNotification(SOUND_EFFECT_VP_WERE_CHASING, 10);
    if (!(StoriesFinished[CurrentPlayer] & STORY_CHASE)) {
      SetGameMode(GAME_MODE_CHASE_TOP_LANES);      
    } else {
      SetGameMode(GAME_MODE_CHASE_POP_BUMPERS);
    }
  } else {
    if (!(StoriesFinished[CurrentPlayer] & STORY_CHASE)) {
      SetGameMode(GameMode+1);
    } else {
      chaseMultiplier = 2;
      switch (GameMode) {
        case GAME_MODE_CHASE_TOP_LANES: SetGameMode(GAME_MODE_CHASE_STANDUPS); break;
        case GAME_MODE_CHASE_POP_BUMPERS: SetGameMode(GAME_MODE_CHASE_TOP_LANES); break;
        case GAME_MODE_CHASE_STANDUPS: SetGameMode(GAME_MODE_CHASE_5_DROPS); break;
        case GAME_MODE_CHASE_LANE_TO_TOP: SetGameMode(GAME_MODE_CHASE_6_DROPS); break;
        case GAME_MODE_CHASE_6_DROPS: SetGameMode(GAME_MODE_CHASE_BULLSEYE); break;
        case GAME_MODE_CHASE_5_DROPS: SetGameMode(GAME_MODE_CHASE_LANE_TO_TOP); break;
        case GAME_MODE_CHASE_BULLSEYE: SetGameMode(GAME_MODE_CHASE_FINAL_SHOT); break;
        case GAME_MODE_CHASE_FINAL_SHOT: SetGameMode(GAME_MODE_CHASE_FINAL_SHOT); break;
      }
    }
  }
  PlaySoundEffect(SOUND_EFFECT_CREATURE_SCREAM_LEFT_1 + (GameMode-GAME_MODE_CHASE_SPINNER));
  StartScoreAnimation(ChaseLegBonus * TotalScoreMultiplier * chaseMultiplier);
}


void ReduceHatchlingProgressOn6Drops() {
  if (HatchlingsProgress&HATCHLING_SPECIAL) {
    HatchlingsProgress &= ~(HATCHLING_SPECIAL);
  } else if (HatchlingsProgress&HATCHLING_50K) {
    HatchlingsProgress &= ~(HATCHLING_50K);
  } else if (HatchlingsProgress&HATCHLING_30K) {
    HatchlingsProgress &= ~(HATCHLING_30K);
  } else if (HatchlingsProgress&HATCHLING_20K) {
    HatchlingsProgress &= ~(HATCHLING_20K);
  } else if (HatchlingsProgress&HATCHLING_10K) {
    HatchlingsProgress &= ~(HATCHLING_10K);
  }
}

void ReduceHatchlingProgressOnLaneToTop() {
  if (HatchlingsProgress&HATCHLING_SPECIAL_RIGHT) {
    HatchlingsProgress &= ~(HATCHLING_SPECIAL_RIGHT);
  } else if (HatchlingsProgress&HATCHLING_EXTRA) {
    HatchlingsProgress &= ~(HATCHLING_EXTRA);
  } else if (HatchlingsProgress&HATCHLING_50K_RIGHT) {
    HatchlingsProgress &= ~(HATCHLING_50K_RIGHT);
  }
}



void HandleGamePlaySwitches(byte switchHit) {

  unsigned long baseScore;
  //LastSwitchHit = switchHit;

  switch (switchHit) {
    case SW_LEFT_BULLSEYE:
      RegisterWeaponsHit(WEAPON_INDEX_RAIL_GUNS, WEAPONS_MODE_RAIL_GUNS, WEAPON_ADVANCE_RAIL_GUNS);
      PlaySoundEffect(SOUND_EFFECT_BULLSEYE_UNLIT);
      AddToBonus(1, 0);
      if (GameMode==GAME_MODE_CHASE_BULLSEYE) AdvanceChase();
      LastSwitchHitTime = CurrentTime;
      LastSwitchHit = switchHit;
      if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
      break;
    case SW_RIGHT_BULLSEYE:
      RegisterWeaponsHit(WEAPON_INDEX_RAIL_GUNS, WEAPONS_MODE_RAIL_GUNS, WEAPON_ADVANCE_RAIL_GUNS);
      PlaySoundEffect(SOUND_EFFECT_BULLSEYE_UNLIT);
      AddToBonus(0, 1);
      if (GameMode==GAME_MODE_CHASE_BULLSEYE) AdvanceChase();
      LastSwitchHitTime = CurrentTime;
      LastSwitchHit = switchHit;
      if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
      break;
    case SW_LEFT_SLING:
    case SW_RIGHT_SLING:
//      if (CurrentTime < (BallSearchSolenoidFireTime[6] + 150)) break;
//      if (CurrentTime < (BallSearchSolenoidFireTime[7] + 150)) break;
      CurrentScores[CurrentPlayer] += TotalScoreMultiplier * 10;
      PlaySoundEffect(SOUND_EFFECT_SLING_SHOT);
      LastSwitchHitTime = CurrentTime;
      LastSwitchHit = switchHit;
      if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
      break;
    case SW_RIGHT_FLIPPER:
//      Audio.PlaySoundCardWhenPossible(18 * 256, CurrentTime);
      if (CurrentTime > (RightFlipperDownTime + 125)) {
        if (GameMode==GAME_MODE_PRE_PLUNGE_CHOICE) {
          WeaponChangedOnce = true;
          CurrentWeaponSelected += 1;
          if (CurrentWeaponSelected>WEAPON_INDEX_GREEN_LASER) CurrentWeaponSelected = WEAPON_INDEX_DEPTH_CHARGES;
          Audio.StopAllNotifications(2);
          if (CurrentWeaponsEngaged[CurrentPlayer]&WeaponIndexToBitmask(CurrentWeaponSelected)) {
            QueueNotification(SOUND_EFFECT_VP_ALREADY_ACTIVE, 2);
            if (WeaponsStatus[CurrentPlayer][CurrentWeaponSelected-1]!=0) {
              OverrideScoreDisplay(CurrentPlayer, WeaponsStatus[CurrentPlayer][CurrentWeaponSelected-1], DISPLAY_OVERRIDE_ANIMATION_CENTER);
              WeaponsStatusShownTime = CurrentTime;
            }
          } else {
            QueueNotification(SOUND_EFFECT_VP_DEPTH_CHARGES + (CurrentWeaponSelected-1), 2);
          }
        } else if (GameMode==GAME_MODE_ADD_LEFT_WEAPON) {
          if (CurrentWeaponSelected>=WEAPON_INDEX_BLUE_LASER) CurrentWeaponSelected = WEAPON_INDEX_DEPTH_CHARGES;
          else CurrentWeaponSelected += 2;
          Audio.StopAllNotifications(2);
          if (CurrentWeaponsEngaged[CurrentPlayer]&WeaponIndexToBitmask(CurrentWeaponSelected)) QueueNotification(SOUND_EFFECT_VP_ALREADY_ACTIVE, 2);
          QueueNotification(SOUND_EFFECT_VP_DEPTH_CHARGES + (CurrentWeaponSelected-1), 2);
          GameModeEndTime = CurrentTime + 3000;
          if (WeaponsStatus[CurrentPlayer][CurrentWeaponSelected-1]!=0) {
            OverrideScoreDisplay(CurrentPlayer, WeaponsStatus[CurrentPlayer][CurrentWeaponSelected-1], DISPLAY_OVERRIDE_ANIMATION_CENTER);
            WeaponsStatusShownTime = CurrentTime;
          }
        } else if (GameMode==GAME_MODE_ADD_RIGHT_WEAPON) {
          if (CurrentWeaponSelected>=WEAPON_INDEX_GREEN_LASER) CurrentWeaponSelected = WEAPON_INDEX_CANNONS;
          else CurrentWeaponSelected += 2;
          Audio.StopAllNotifications(2);
          if (CurrentWeaponsEngaged[CurrentPlayer]&WeaponIndexToBitmask(CurrentWeaponSelected)) QueueNotification(SOUND_EFFECT_VP_ALREADY_ACTIVE, 2);
          QueueNotification(SOUND_EFFECT_VP_DEPTH_CHARGES + (CurrentWeaponSelected-1), 2);
          GameModeEndTime = CurrentTime + 3000;
          if (WeaponsStatus[CurrentPlayer][CurrentWeaponSelected-1]!=0) {
            OverrideScoreDisplay(CurrentPlayer, WeaponsStatus[CurrentPlayer][CurrentWeaponSelected-1], DISPLAY_OVERRIDE_ANIMATION_CENTER);
            WeaponsStatusShownTime = CurrentTime;
          }
        } else if (GameMode==GAME_MODE_SKILL_SHOT) {
          SkillShotLane*=2;
          if (SkillShotLane==0x08) SkillShotLane=1;
        } else {
          CountdownBonusHurryUp = true;
          TopLaneStatus[CurrentPlayer] *= 2;
          if (TopLaneStatus[CurrentPlayer]&0x08) {
            TopLaneStatus[CurrentPlayer] -= 0x07;
          }
        }
        RightFlipperDownTime = CurrentTime;
      }
      break;
    case SW_4_ROLLOVER:
    case SW_5_ROLLOVER:
    case SW_6_ROLLOVER:
      HandleTopLaneSwitches(switchHit);
      if (GameMode==GAME_MODE_CHASE_TOP_LANES) AdvanceChase();
      LastSwitchHitTime = CurrentTime;
      LastSwitchHit = switchHit;
      if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
      break;
    case SW_RIGHT_LANE_TO_TOP:
      if (LaneToTopLastHit==0 || CurrentTime>(LaneToTopLastHit+500)) {
        LaneToTopLastHit = CurrentTime;
        ReduceHatchlingProgressOnLaneToTop();
        if (RegisterWeaponsHit(WEAPON_INDEX_AIR_ASSAULT, WEAPONS_MODE_AIR_ASSAULT, WEAPON_ADVANCE_AIR_ASSAULT*2)) {
          PlaySoundEffect(SOUND_EFFECT_TOP_LANE_LEVEL_FINISHED);
        } else {
          if (LaneToTopValue[CurrentPlayer]==0) {
            CurrentScores[CurrentPlayer] += 100 * TotalScoreMultiplier;
            PlaySoundEffect(SOUND_EFFECT_TOP_LANE_REPEAT);
          }
        }
        if (LaneToTopValue[CurrentPlayer]==1) {
          StartScoreAnimation(50000 * TotalScoreMultiplier);
          PlaySoundEffect(SOUND_EFFECT_SKILL_SHOT);
          LaneToTopValue[CurrentPlayer] = 0;
        } else if (LaneToTopValue[CurrentPlayer]==2) {
          AwardExtraBall();
          LaneToTopValue[CurrentPlayer] = 1;
        } else if (LaneToTopValue[CurrentPlayer]==3) {
          AwardSpecial();
          LaneToTopValue[CurrentPlayer] = 2;
        }
      }
      if (GameMode==GAME_MODE_CHASE_LANE_TO_TOP) AdvanceChase();
      LastSwitchHitTime = CurrentTime;
      LastSwitchHit = switchHit;
      if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
      break;
    case SW_CENTER_RIGHT_10:
      if (GameMode==GAME_MODE_PRE_PLUNGE_CHOICE) {
        SuperSkillShotReady = true;
        SetGameMode(GAME_MODE_SKILL_SHOT);
      } else {
        LastSwitchHitTime = CurrentTime;
        LastSwitchHit = switchHit;
      }
      PlaySoundEffect(SOUND_EFFECT_LOWER_BUMPER_HIT);
      break;
    case SW_SPINNER:
      if (GameMode==GAME_MODE_PRE_PLUNGE_CHOICE) {
        SuperSkillShotReady = true;
        SetGameMode(GAME_MODE_SKILL_SHOT);
      } else {
        LastSwitchHitTime = CurrentTime;
        LastSwitchHit = switchHit;
      }
      baseScore = 100;
      if (TopLaneLevel[CurrentPlayer]>=1 && StandupLevel[CurrentPlayer]>=1) {
        baseScore = 1000;
        TotalPopHits[CurrentPlayer][0] += 1;
        if (TotalPopHits[CurrentPlayer][0]>=POP_HITS_FOR_HOLDOVER) {
          if (!(HoldoverAwards[CurrentPlayer]&HOLDOVER_MULTIBALL_TIMER)) {
            QueueNotification(SOUND_EFFECT_VP_HOLDOVER_MULTIBALL_TIMER, 8);
          }
          HoldoverAwards[CurrentPlayer] |= HOLDOVER_MULTIBALL_TIMER;
          TotalPopHits[CurrentPlayer][0] = POP_HITS_FOR_HOLDOVER;
        } else {
          for (byte count=0; count<4; count++) {
            if (count!=CurrentPlayer) OverrideScoreDisplay(count, TotalPopHits[CurrentPlayer][0], DISPLAY_OVERRIDE_ANIMATION_FLYBY);
          }
        }
      }
      HatchlingsProgress &= ~(HATCHLING_SPINNER);
      RegisterWeaponsHit(WEAPON_INDEX_CANNONS, WEAPONS_MODE_CANNONS, 1);
      CurrentScores[CurrentPlayer] += baseScore * TotalScoreMultiplier;
      PlaySoundEffect(SOUND_EFFECT_SPINNER_ALT);
      if (GameMode==GAME_MODE_CHASE_SPINNER) AdvanceChase();
      if (GameMode==GAME_MODE_CHASE_START) SetGameMode(GAME_MODE_CHASE_SPINNER);
      break;
    case SW_LEFT_3_DROP_1:
    case SW_LEFT_3_DROP_2:
    case SW_LEFT_3_DROP_3:
    case SW_RIGHT_3_DROP_1:
    case SW_RIGHT_3_DROP_2:
    case SW_RIGHT_3_DROP_3:
    case SW_3_DROPS_COMPLETE:
      if (Handle6Drops(switchHit)) {
        ReduceHatchlingProgressOn6Drops();
        if (GameMode==GAME_MODE_CHASE_6_DROPS) AdvanceChase();
        LastSwitchHitTime = CurrentTime;
        LastSwitchHit = switchHit;
        if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
      }
      break;
    case SW_5_DROP_1:
    case SW_5_DROP_2:
    case SW_5_DROP_3:
    case SW_5_DROP_4:
    case SW_5_DROP_5:
    case SW_5_DROP_COMPLETE:
      if (Handle5Drops(switchHit)) {
        if (GameMode==GAME_MODE_CHASE_5_DROPS) AdvanceChase();
        HatchlingsProgress &= ~(HATCHLING_LITES_OUT);
        LastSwitchHitTime = CurrentTime;
        LastSwitchHit = switchHit;
        if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
      }
      break;
    case SW_LEFT_OUTLANE:
    case SW_RIGHT_OUTLANE:
      if (CurrentTime<(LastOutlaneHit+OUT_LANE_DEBOUNCE_TIME)) break;
      LastOutlaneHit = CurrentTime;
      if (BallSaveEndTime) BallSaveEndTime += 3000;
      PlaySoundEffect(SOUND_EFFECT_OUTLANE_UNLIT);
      CurrentScores[CurrentPlayer] += TotalScoreMultiplier * 5000;
      LastSwitchHitTime = CurrentTime;
      LastSwitchHit = switchHit;
      if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
      break;
    case SW_LEFT_CENTER_INLANE:
      if (CurrentTime<(LastInlaneHit[0]+IN_LANE_DEBOUNCE_TIME)) break;
      LastInlaneHit[0] = CurrentTime;    
      PlaySoundEffect(SOUND_EFFECT_OUTLANE_LIT);
      CurrentScores[CurrentPlayer] += TotalScoreMultiplier * 1000;
      AddToBonus(1, 0);
      LastSwitchHitTime = CurrentTime;
      LastSwitchHit = switchHit;
      if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
      break;
    case SW_LEFT_INSIDE_INLANE:
      if (CurrentTime<(LastInlaneHit[1]+IN_LANE_DEBOUNCE_TIME)) break;
      LastInlaneHit[1] = CurrentTime;    
      PlaySoundEffect(SOUND_EFFECT_OUTLANE_LIT);
      CurrentScores[CurrentPlayer] += TotalScoreMultiplier * 1000;
      AddToBonus(0, 1);
      LastSwitchHitTime = CurrentTime;
      LastSwitchHit = switchHit;
      if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
      break;
    case SW_RIGHT_INSIDE_INLANE:
      if (CurrentTime<(LastInlaneHit[2]+IN_LANE_DEBOUNCE_TIME)) break;
      LastInlaneHit[2] = CurrentTime;    
      PlaySoundEffect(SOUND_EFFECT_OUTLANE_LIT);
      CurrentScores[CurrentPlayer] += TotalScoreMultiplier * 1000;
      AddToBonus(1, 0);
      LastSwitchHitTime = CurrentTime;
      LastSwitchHit = switchHit;
      if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
      break;
    case SW_RIGHT_CENTER_INLANE:
      if (CurrentTime<(LastInlaneHit[3]+IN_LANE_DEBOUNCE_TIME)) break;
      LastInlaneHit[3] = CurrentTime;    
      PlaySoundEffect(SOUND_EFFECT_OUTLANE_LIT);
      CurrentScores[CurrentPlayer] += TotalScoreMultiplier * 1000;
      AddToBonus(0, 1);
      LastSwitchHitTime = CurrentTime;
      LastSwitchHit = switchHit;
      if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
      break;
    case SW_1_TARGET:
    case SW_2_TARGET:
    case SW_3_TARGET:
      HandleStandupSwitches(switchHit);
      if (GameMode==GAME_MODE_CHASE_STANDUPS) AdvanceChase();
      LastSwitchHitTime = CurrentTime;
      LastSwitchHit = switchHit;
      if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
      break;
    case SW_RIGHT_POP_BUMPER:
      if (LastRightPopHit && CurrentTime<(LastRightPopHit+POP_BUMPER_DEBOUNCE_TIME)) break;// debounce noise
      HatchlingsProgress &= ~(HATCHLING_RIGHT_POP);
      LastRightPopHit = CurrentTime;
      PlaySoundEffect(SOUND_EFFECT_BUMPER_HIT);
      if (GameMode==GAME_MODE_CHASE_POP_BUMPERS) AdvanceChase();
      LastSwitchHitTime = CurrentTime;
      LastSwitchHit = switchHit;
      RegisterWeaponsHit(WEAPON_INDEX_CANNONS, WEAPONS_MODE_CANNONS, WEAPON_ADVANCE_CANNONS);
      baseScore = 100;
      if (TopLaneLevel[CurrentPlayer]>=2 && StandupLevel[CurrentPlayer]>=2) {
        baseScore = 1000;
        TotalPopHits[CurrentPlayer][1] += 1;
        if (TotalPopHits[CurrentPlayer][1]>=POP_HITS_FOR_HOLDOVER) {
          if (!(HoldoverAwards[CurrentPlayer]&HOLDOVER_BONUS)) {
            QueueNotification(SOUND_EFFECT_VP_HOLDOVER_BONUS, 8);
          }
          HoldoverAwards[CurrentPlayer] |= HOLDOVER_BONUS;
          TotalPopHits[CurrentPlayer][1] = POP_HITS_FOR_HOLDOVER;
        } else {
          for (byte count=0; count<4; count++) {
            if (count!=CurrentPlayer) OverrideScoreDisplay(count, TotalPopHits[CurrentPlayer][1], DISPLAY_OVERRIDE_ANIMATION_FLYBY);
          }
        }
      }
      CurrentScores[CurrentPlayer] += baseScore * TotalScoreMultiplier;      
      if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
      break;
    case SW_UPPER_POP_BUMPER:
      if (LastUpperPopHit && CurrentTime<(LastUpperPopHit+POP_BUMPER_DEBOUNCE_TIME)) break;// debounce noise
      HatchlingsProgress &= ~(HATCHLING_UPPER_POP);
      if (GameMode==GAME_MODE_CHASE_POP_BUMPERS) AdvanceChase();
      LastUpperPopHit = CurrentTime;
      PlaySoundEffect(SOUND_EFFECT_BUMPER_HIT);
      LastSwitchHitTime = CurrentTime;
      LastSwitchHit = switchHit;
      RegisterWeaponsHit(WEAPON_INDEX_CANNONS, WEAPONS_MODE_CANNONS, WEAPON_ADVANCE_CANNONS);
      baseScore = 100;
      if (TopLaneLevel[CurrentPlayer]>=3 && StandupLevel[CurrentPlayer]>=3) {
        baseScore = 1000;
        TotalPopHits[CurrentPlayer][2] += 1;
        if (TotalPopHits[CurrentPlayer][2]>=POP_HITS_FOR_HOLDOVER) {
          if (!(HoldoverAwards[CurrentPlayer]&HOLDOVER_BONUS_X)) {
            QueueNotification(SOUND_EFFECT_VP_HOLDOVER_BONUS_X, 8);
          }
          HoldoverAwards[CurrentPlayer] |= HOLDOVER_BONUS_X;
          TotalPopHits[CurrentPlayer][2] = POP_HITS_FOR_HOLDOVER;
        } else {
          for (byte count=0; count<4; count++) {
            if (count!=CurrentPlayer) OverrideScoreDisplay(count, TotalPopHits[CurrentPlayer][2], DISPLAY_OVERRIDE_ANIMATION_FLYBY);
          }
        }
      }
      CurrentScores[CurrentPlayer] += baseScore * TotalScoreMultiplier;
      if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
      break;
    case SW_LOWER_POP_BUMPER:
      if (LastLowerPopHit && CurrentTime<(LastLowerPopHit+POP_BUMPER_DEBOUNCE_TIME)) break;// debounce noise
      HatchlingsProgress &= ~(HATCHLING_SPINNER);
      if (GameMode==GAME_MODE_CHASE_POP_BUMPERS) AdvanceChase();
      if (GameMode==GAME_MODE_CHASE_SPINNER && CurrentTime>(ChasePromptPlayed+10000)) {
        QueueNotification(SOUND_EFFECT_VP_SPINNER_STARTS_CHASE_2, 10);
        ChasePromptPlayed = CurrentTime;
      }
      baseScore = 100;
      if (TopLaneLevel[CurrentPlayer]>=1 && StandupLevel[CurrentPlayer]>=1) {
        baseScore = 1000;
        TotalPopHits[CurrentPlayer][0] += 1;
        if (TotalPopHits[CurrentPlayer][0]>=POP_HITS_FOR_HOLDOVER) {
          if (!(HoldoverAwards[CurrentPlayer]&HOLDOVER_MULTIBALL_TIMER)) {
            QueueNotification(SOUND_EFFECT_VP_HOLDOVER_MULTIBALL_TIMER, 8);
          }
          HoldoverAwards[CurrentPlayer] |= HOLDOVER_MULTIBALL_TIMER;
          TotalPopHits[CurrentPlayer][0] = POP_HITS_FOR_HOLDOVER;
        } else {
          for (byte count=0; count<4; count++) {
            if (count!=CurrentPlayer) OverrideScoreDisplay(count, TotalPopHits[CurrentPlayer][0], DISPLAY_OVERRIDE_ANIMATION_FLYBY);
          }
        }
      }
      LastLowerPopHit = CurrentTime;
      CurrentScores[CurrentPlayer] += baseScore * TotalScoreMultiplier;
      PlaySoundEffect(SOUND_EFFECT_BUMPER_HIT);
      LastSwitchHitTime = CurrentTime;
      LastSwitchHit = switchHit;
      RegisterWeaponsHit(WEAPON_INDEX_CANNONS, WEAPONS_MODE_CANNONS, WEAPON_ADVANCE_CANNONS);
      if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
      break;
    case SW_UPPER_RIGHT_10:
/*      CurrentScores[CurrentPlayer] += 100 * TotalScoreMultiplier;
      LastSwitchHitTime = CurrentTime;
      if (BallFirstSwitchHitTime == 0) BallFirstSwitchHitTime = CurrentTime;
*/      
      break;
    case SW_LEFT_SAUCER:
      break;
    case SW_RIGHT_SAUCER:
      break;
  }


}


int RunGamePlayMode(int curState, boolean curStateChanged) {
  int returnState = curState;
  unsigned long scoreAtTop = CurrentScores[CurrentPlayer];

  // Very first time into gameplay loop
  if (curState == MACHINE_STATE_INIT_GAMEPLAY) {
    returnState = InitGamePlay(curStateChanged);
  } else if (curState == MACHINE_STATE_INIT_NEW_BALL) {
    returnState = InitNewBall(curStateChanged, CurrentPlayer, CurrentBallInPlay);
  } else if (curState == MACHINE_STATE_NORMAL_GAMEPLAY) {
    returnState = ManageGameMode();
  } else if (curState == MACHINE_STATE_COUNTDOWN_BONUS) {
    returnState = CountdownBonus(curStateChanged);
    ShowPlayerScores(0xFF, false, false);
  } else if (curState == MACHINE_STATE_BALL_OVER) {
    WOS_SetDisplayCredits(Credits, !FreePlayMode);

    if (SamePlayerShootsAgain) {
      QueueNotification(SOUND_EFFECT_VP_SHOOT_AGAIN, 10);
      returnState = MACHINE_STATE_INIT_NEW_BALL;
    } else {

      CurrentPlayer += 1;
      if (CurrentPlayer >= CurrentNumPlayers) {
        CurrentPlayer = 0;
        CurrentBallInPlay += 1;
      }

      scoreAtTop = CurrentScores[CurrentPlayer];

      if (CurrentBallInPlay > BallsPerGame) {
        CheckHighScores();
        PlaySoundEffect(SOUND_EFFECT_GAME_OVER);
        for (int count = 0;count < CurrentNumPlayers;count++) {
          WOS_SetDisplay(count, CurrentScores[count], true, 2);
        }

        returnState = MACHINE_STATE_MATCH_MODE;
      }
      else returnState = MACHINE_STATE_INIT_NEW_BALL;
    }
  } else if (curState == MACHINE_STATE_MATCH_MODE) {
    returnState = ShowMatchSequence(curStateChanged);
  }

  byte switchHit;
  unsigned long lastBallFirstSwitchHitTime = BallFirstSwitchHitTime;

  while ( (switchHit = WOS_PullFirstFromSwitchStack()) != SWITCH_STACK_EMPTY ) {
    returnState = HandleSystemSwitches(curState, switchHit);
    if (NumTiltWarnings <= MaxTiltWarnings && MachineState==MACHINE_STATE_NORMAL_GAMEPLAY) HandleGamePlaySwitches(switchHit);
  }

  UpdateSaucerStatus(curState);

  if (CreditResetPressStarted) {
    if (CurrentBallInPlay < 2) {
      // If we haven't finished the first ball, we can add players
      AddPlayer();
      CreditResetPressStarted = 0;
    } else {
      if (WOS_ReadSingleSwitchState(SW_CREDIT_RESET)) {

        if (TimeRequiredToResetGame != 99 && (CurrentTime - CreditResetPressStarted) >= ((unsigned long)TimeRequiredToResetGame*1000)) {
          // If the first ball is over, pressing start again resets the game
          if (Credits >= 1 || FreePlayMode) {
            if (!FreePlayMode) {
              Credits -= 1;
              WOS_WriteByteToEEProm(WOS_CREDITS_EEPROM_BYTE, Credits);
              WOS_SetDisplayCredits(Credits, !FreePlayMode);
            }
            returnState = MACHINE_STATE_INIT_GAMEPLAY;
            CreditResetPressStarted = 0;
          }
        }
      } else {
        CreditResetPressStarted = 0;
      }
    }

  }  
  
  if (!WOS_ReadSingleSwitchState(SW_RIGHT_FLIPPER)) {
    RightFlipperDownTime = 0;
    RightFlipperHeld = false;
    RightFlipperHeldLong = false;
  } else {
    if (CurrentTime > (RightFlipperDownTime+5000)) {
      RightFlipperHeldLong = true;
    } else if (CurrentTime > (RightFlipperDownTime+1000)) {
      RightFlipperHeld = true;
    }
  }
  
  if (lastBallFirstSwitchHitTime==0 && BallFirstSwitchHitTime!=0) {
    BallSaveEndTime = BallFirstSwitchHitTime + ((unsigned long)BallSaveNumSeconds)*1000;
  }
  if (CurrentTime>(BallSaveEndTime+BALL_SAVE_GRACE_PERIOD)) {
    BallSaveEndTime = 0;
  }

  if (!ScrollingScores && CurrentScores[CurrentPlayer] > WOS_MAX_DISPLAY_SCORE) {
    CurrentScores[CurrentPlayer] -= WOS_MAX_DISPLAY_SCORE;
    if (!TournamentScoring) AddSpecialCredit();
  }

  if (scoreAtTop != CurrentScores[CurrentPlayer]) {
    LastTimeScoreChanged = CurrentTime;
    if (!TournamentScoring) {
      for (int awardCount = 0;awardCount < 3;awardCount++) {
        if (AwardScores[awardCount] != 0 && scoreAtTop < AwardScores[awardCount] && CurrentScores[CurrentPlayer] >= AwardScores[awardCount]) {
          // Player has just passed an award score, so we need to award it
          if (((ScoreAwardReplay >> awardCount) & 0x01)) {
            AddSpecialCredit();
          } else if (!ExtraBallCollected) {
            AwardExtraBall();
          }
        }
      }
    }

  }

  return returnState;
}


byte AudioInit = 0;

void InitAudio() {
  if (CurrentTime>1500 && AudioInit==0) {
    if (DEBUG_MESSAGES) {
      Serial.write("About to init Audio\n");
    }
    AudioInit = 1;
  } else if (CurrentTime>1600) {
    if (AudioInit==1) {
      AudioInit = 2;
      if (DEBUG_MESSAGES) {
        Serial.write("Updating audio for the first time\n");
      }
    }
    Audio.Update(CurrentTime);
  }
  
}


unsigned long LastLEDUpdateTime = 0;
byte LEDPhase = 0;
unsigned long NumLoops = 0;
unsigned long LastLoopReportTime = 0;

void loop() {

  if (DEBUG_MESSAGES) {
    NumLoops += 1;
    if (/*LastLoopReportTime==0 || */CurrentTime>(LastLoopReportTime+1000)) {
      LastLoopReportTime = CurrentTime;
      char buf[128];
      sprintf(buf, "Loop running at %lu Hz, %d\n", NumLoops, MachineState);
      Serial.write(buf);
      NumLoops = 0;
    }
  }
  
  int newMachineState = MachineState;
  CurrentTime = millis();

  if (MachineState < 0) {
    newMachineState = RunSelfTest(MachineState, MachineStateChanged);
  } else if (MachineState == MACHINE_STATE_ATTRACT) { 
    newMachineState = RunAttractMode(MachineState, MachineStateChanged);
  } else if (MachineState == MACHINE_STATE_DIAGNOSTICS) {
    newMachineState = RunDiagnosticsMode(MachineState, MachineStateChanged);
  } else {
    if (DEBUG_MESSAGES) {
      Serial.write("Would have called RunGamePlayMode\n");
    }
    newMachineState = RunGamePlayMode(MachineState, MachineStateChanged);
  } 

  if (newMachineState != MachineState) {
    MachineState = newMachineState;
    MachineStateChanged = true;
  } else {
    MachineStateChanged = false; 
  }


  if (1) {
    WOS_ApplyFlashToLamps(CurrentTime);
    WOS_UpdateTimedSolenoidStack(CurrentTime);
    WOS_UpdateTimedSoundStack(CurrentTime);
  } else {
    WOS_Update(CurrentTime);
  }

  //InitAudio();
  
  if (LastLEDUpdateTime == 0 || (CurrentTime - LastLEDUpdateTime) > 250) {
    LastLEDUpdateTime = CurrentTime;
    WOS_SetBoardLEDs((LEDPhase % 8) == 0 || (LEDPhase % 8) == 3, (LEDPhase % 8) == 5 || (LEDPhase % 8) == 7);
    LEDPhase += 1;
  }
   
}
