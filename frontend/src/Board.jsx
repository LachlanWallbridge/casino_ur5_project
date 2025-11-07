import React, { useEffect, useMemo, useState } from 'react';
import { Button } from 'react-bootstrap';
import {
    callStartRoundService,
    subscribeToRoundResult,
    subscribeToDiceResults
} from './ui_bridge';
import 'bootstrap/dist/css/bootstrap.min.css';

// ====== CONFIG ======
// ====== CONFIG ======
// Set your actual chip values here:
const COLOR_VALUE = {
  red:   10,
  white: 10,
  blue:  10,
  green: 10,
};

function Board({ players = [] }) {
    const [roundActive, setRoundActive] = useState(false);
    const [resultText, setResultText] = useState('');
    const [dice, setDice] = useState([]);

    // Subscribe to dice updates
    useEffect(() => {
        const unsub = subscribeToDiceResults((msg) => {
            setDice(Array.isArray(msg?.dice) ? msg.dice : []);
        });
        return () => unsub();
    }, []);

    // Compute sum and parity
    const diceSum = useMemo(
        () => dice.reduce((s, d) => s + (d?.dice_number || 0), 0),
        [dice]
    );
    const diceParityLabel = useMemo(() => {
        if (!dice.length) return '—';
        return diceSum % 2 === 0 ? 'EVEN' : 'ODD';
    }, [dice, diceSum]);

    // Subscribe to round completion
    useEffect(() => {
        const unsub = subscribeToRoundResult(async (msg) => {
            if (!msg || !msg.is_complete) return;

            setRoundActive(false);
            const isOdd = diceSum % 2 === 1;
            const label = isOdd ? 'ODD' : 'EVEN';
            setResultText(`Result: ${label}`);

            try {
                await settleAllPlayers(players, isOdd);
            } catch (err) {
                console.error('Payout error:', err);
            }
        });
        return () => unsub();
    }, [diceSum, players]);

    const startGame = () => {
        setRoundActive(true);
        setResultText('');
        callStartRoundService((resp) => {
            if (!resp || !resp.accepted) {
                setRoundActive(false);
                alert('Round not accepted: ' + (resp?.message || 'unknown error'));
            }
        });
    };

    // ===== STYLING =====
    const boardStyle = {
        width: '100%',
        height: '100%',
        background: 'radial-gradient(circle at 30% 30%, #0b6623 0%, #054d1b 80%)',
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'center',
        borderRadius: '20px',
        border: '6px solid #d4af37',
        boxShadow: '0 0 25px rgba(0,0,0,0.4), inset 0 0 20px rgba(0,0,0,0.3)',
        fontFamily: "'Cinzel Decorative', serif",
        color: '#fff',
        textShadow: '0 0 6px rgba(0,0,0,0.6)',
        position: 'relative',
        overflow: 'hidden',
    };

    const feltOverlay = {
        content: "''",
        position: 'absolute',
        top: 0,
        left: 0,
        width: '100%',
        height: '100%',
        backgroundImage:
            'repeating-linear-gradient(45deg, rgba(255,255,255,0.03) 0 2px, transparent 2px 6px)',
        pointerEvents: 'none',
    };

    return (
        <div className="container-fluid p-3" style={{ height: '100vh', background: '#222' }}>
            <div style={boardStyle}>
                <div style={feltOverlay}></div>

                {/* Title */}
                <h2
                    style={{
                        fontSize: '3rem',
                        color: '#ffd700',
                        textShadow: '0 0 12px rgba(255, 215, 0, 0.7)',
                    }}
                >
                    🎲 Place Your Bets
                </h2>

                {/* Dice */}
                <div className="d-flex align-items-center gap-3 mt-3">
                    {dice.length ? (
                        dice.map((d, i) => (
                            <DiceTile key={i} value={d.dice_number} conf={d.confidence} />
                        ))
                    ) : (
                        <span className="text-light">Waiting for dice…</span>
                    )}
                </div>

                {/* Sum / parity */}
                <div className="mt-3" style={{ fontSize: '1.2rem' }}>
                    Sum: <strong>{dice.length ? diceSum : '—'}</strong> &nbsp;|&nbsp; Parity:{' '}
                    <strong>{diceParityLabel}</strong>
                </div>

                {/* Round status */}
                <p style={{ fontSize: '1.2rem', color: '#eaeaea', marginTop: '0.75rem' }}>
                    {resultText || (roundActive ? 'Round in progress...' : 'Ready to start!')}
                </p>

                {/* Start button */}
                {!roundActive && (
                    <Button
                        variant="warning"
                        size="lg"
                        className="mt-3"
                        style={{
                            fontWeight: 'bold',
                            fontSize: '1.5rem',
                            padding: '0.6em 1.6em',
                            borderRadius: '12px',
                            boxShadow: '0 0 15px rgba(255,215,0,0.4)',
                            color: '#222',
                        }}
                        onClick={startGame}
                    >
                        Start Game
                    </Button>
                )}
            </div>
        </div>
    );
}

export default Board;

// ===== Dice visual =====
function DiceTile({ value = 0, conf = 0 }) {
    return (
        <div
            className="d-flex flex-column align-items-center"
            style={{ minWidth: '64px' }}
        >
            <div
                className="d-flex align-items-center justify-content-center"
                style={{
                    width: '64px',
                    height: '64px',
                    background: '#fff',
                    color: '#000',
                    borderRadius: '12px',
                    border: '3px solid #222',
                    boxShadow: '0 4px 14px rgba(0,0,0,0.35)',
                    fontWeight: 800,
                    fontSize: '1.75rem',
                }}
            >
                {value || '—'}
            </div>
            <small className="text-light mt-1" style={{ opacity: 0.8 }}>
                conf {(conf ?? 0).toFixed(2)}
            </small>
        </div>
    );
}

// ===== Payout helpers =====
async function settleAllPlayers(players, isOdd) {
  const tasks = players.map((p) => settleOnePlayer(p, isOdd));
  await Promise.all(tasks);
}

// Reads bet parity robustly from player object.
// Accepts: player.bet_parity = "odd" | "even"  OR player.bet_is_odd = boolean
function readPlayerBetParity(player) {
  if (typeof player?.bet_is_odd === 'boolean') return player.bet_is_odd;
  const s = (player?.bet_parity || '').toString().toLowerCase();
  if (s === 'odd') return true;
  if (s === 'even') return false;
  return null; // unknown
}

function computeWagerFromColors(colors) {
  return (colors || [])
    .map((c) => (c || '').toString().toLowerCase())
    .reduce((sum, c) => sum + (COLOR_VALUE[c] || 0), 0);
}

async function settleOnePlayer(player, isOdd) {
  // Read the player's declared bet side
  const playerBetIsOdd = readPlayerBetParity(player);
  if (playerBetIsOdd === null) {
    console.warn(`Player ${player?.player_id}: no bet parity set; skipping payout.`);
    return;
  }

  // Sum wager from chip colours
  const wager = computeWagerFromColors(player?.bet_colors);
  if (wager <= 0) {
    // No chips placed → no balance change, but still count a game played
    await fetch(`/players/${player.player_id}/game`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ won: false }),
    });
    return;
  }

  const won = (playerBetIsOdd === isOdd);
  const delta = won ? +wager : -wager;

  const gameReq = fetch(`/players/${player.player_id}/game`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ won }),
  });

  const balanceReq = fetch(`/players/${player.player_id}/balance`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ amount: delta }),
  });

  await Promise.all([gameReq, balanceReq]);
}

