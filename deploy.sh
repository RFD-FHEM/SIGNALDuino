#!/bin/bash
set -e

#if [ "$TRAVIS_TAG" =~ "^release.*$" OR "$TRAVIS_TAG" =~ "^R\d\.\d.*$" AND "$TRAVIC_BRANCH" = "dev-r33_cc1101"  ]
if [[ ${TRAVIS_TAG} =~ ^release.*$  || $TRAVIS_TAG =~ ^R\d\.\d.*$ && "$TRAVIC_BRANCH" = "dev-r33_cc1101" ]]
  then
    curl --location "https://github.com/tfausak/github-release/releases/download/1.1.4/github-release-1.1.4-$TRAVIS_OS_NAME.gz" > github-release.gz
    gunzip github-release.gz
    ./github-release upload \
      --token "$GH_API_KEY" \
      --owner Sidey79 \
      --repo RFD-FHEM/SIGNALDuino \
      --tag "$TRAVIS_TAG" \
      --file "$PWD/release/SIGNALDuino_${BOARD}${RECEIVER}${TRAVIS_TAG}.hex" \
      --name "SIGNALDuino_${BOARD}${RECEIVER}${TRAVIS_TAG}"
  fi
  
#if [[ $TRAVIS_TAG =~ ^nightly.*$  ]]
#  then
    DATE=`date +%Y-%m-%d`
    curl --location "https://github.com/tfausak/github-release/releases/download/1.1.4/github-release-1.1.4-$TRAVIS_OS_NAME.gz" > github-release.gz
    gunzip github-release.gz
	chmod +x ./github-release
    ./github-release upload \
      --token "$GH_API_KEY" \
      --owner Sidey79 \
      --repo RFD-FHEM/SIGNALDuino \
      --file "$PWD/release/SIGNALDuino_${BOARD}${RECEIVER}${TRAVIS_TAG}.hex" \
      --name "nightly-SIGNALDuino_${BOARD}${RECEIVER}${DATE}"
#      --tag "$TRAVIS_TAG" \
#  fi