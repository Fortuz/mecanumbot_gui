/**
 * test_js_map_controls.js — Node.js unit tests for the pure-JS helper
 * functions in map_controls.html.
 *
 * The functions are extracted here and tested with a minimal hand-rolled
 * test runner so no external test framework is required.
 *
 * Run with:
 *   node tests/test_js_map_controls.js
 *
 * Exit code 0 = all pass, non-zero = failures.
 */

"use strict";

// ── Minimal test runner ────────────────────────────────────────────────────

let _passed = 0;
let _failed = 0;

function test(label, fn) {
    try {
        fn();
        console.log(`  ✓ ${label}`);
        _passed++;
    } catch (e) {
        console.error(`  ✗ ${label}`);
        console.error(`      ${e.message}`);
        _failed++;
    }
}

function expect(actual) {
    return {
        toBe(expected) {
            if (actual !== expected) {
                throw new Error(`Expected ${JSON.stringify(expected)}, got ${JSON.stringify(actual)}`);
            }
        },
        toContain(substr) {
            if (!String(actual).includes(substr)) {
                throw new Error(`Expected ${JSON.stringify(actual)} to contain ${JSON.stringify(substr)}`);
            }
        },
        notToContain(substr) {
            if (String(actual).includes(substr)) {
                throw new Error(`Expected ${JSON.stringify(actual)} NOT to contain ${JSON.stringify(substr)}`);
            }
        },
        toEqual(expected) {
            const a = JSON.stringify(actual);
            const b = JSON.stringify(expected);
            if (a !== b) {
                throw new Error(`Expected ${b}, got ${a}`);
            }
        },
    };
}

// ── Functions under test (copied verbatim from map_controls.html) ──────────

function escHtml(s) {
    return String(s)
        .replace(/&/g,  '&amp;')
        .replace(/</g,  '&lt;')
        .replace(/>/g,  '&gt;')
        .replace(/"/g,  '&quot;')
        .replace(/'/g,  '&#39;');
}

// ── escHtml tests ──────────────────────────────────────────────────────────

console.log("\nescHtml");

test("escapes ampersand", () => {
    expect(escHtml("a&b")).toBe("a&amp;b");
});
test("escapes less-than", () => {
    expect(escHtml("a<b")).toBe("a&lt;b");
});
test("escapes greater-than", () => {
    expect(escHtml("a>b")).toBe("a&gt;b");
});
test("escapes double quote", () => {
    expect(escHtml('say "hi"')).toBe("say &quot;hi&quot;");
});
test("escapes single quote / apostrophe", () => {
    expect(escHtml("it's")).toBe("it&#39;s");
});
test("escapes all special chars together", () => {
    expect(escHtml(`<b id="x" class='y'>&`)).toBe("&lt;b id=&quot;x&quot; class=&#39;y&#39;&gt;&amp;");
});
test("leaves plain text unchanged", () => {
    expect(escHtml("hello world")).toBe("hello world");
});
test("converts non-string via String()", () => {
    expect(escHtml(42)).toBe("42");
    expect(escHtml(null)).toBe("null");
});
test("mapping name with apostrophe escapes correctly", () => {
    // Regression: onclick='applyMapping(${escHtml(JSON.stringify(name))},...)' would break
    // if escHtml didn't escape single quotes.
    const name = "Alice's Map";
    const escaped = escHtml(JSON.stringify(name));
    // Must not contain a raw single quote that would terminate the onclick attribute
    expect(escaped).notToContain("'");
    expect(escaped).toContain("&#39;");
});

// ── onclick attribute safety ───────────────────────────────────────────────

console.log("\nonclick attribute generation");

function makeOnclickAttr(fnName, name, side) {
    return `onclick='${fnName}(${escHtml(JSON.stringify(name))},${JSON.stringify(side)})'`;
}

test("simple name generates valid onclick", () => {
    const attr = makeOnclickAttr("applyMapping", "Teleop Default", "host");
    expect(attr).toBe(`onclick='applyMapping("Teleop Default","host")'`);
});
test("apostrophe in name does not break onclick attribute", () => {
    const attr = makeOnclickAttr("applyMapping", "Driver's Setup", "host");
    // Must not have an unescaped single quote inside the onclick value
    const inner = attr.slice(`onclick='`.length, attr.length - 1);
    expect(inner).notToContain("'");
});
test("double-quote in name is escaped inside single-quoted attribute", () => {
    const attr = makeOnclickAttr("modifyMapping", 'My "Best" Map', "robot");
    const inner = attr.slice(`onclick='`.length, attr.length - 1);
    expect(inner).toContain("&quot;");
    expect(inner).notToContain('"');   // raw double-quote would still be safe here,
                                        // but we verify escHtml handles it anyway
});
test("side value is passed as JSON string", () => {
    const attr = makeOnclickAttr("deleteMapping", "Map", "robot");
    expect(attr).toContain('"robot"');
});

// ── collectForm-equivalent logic: trigger_mode derivation ─────────────────

console.log("\ntrigger_mode derivation");

// Simulate the logic from collectForm()
function deriveTriggerMode(actionType) {
    return actionType === "button_hold" ? "hold" : "once";
}

test("button_hold action type → trigger_mode 'hold'", () => {
    expect(deriveTriggerMode("button_hold")).toBe("hold");
});
test("button_once action type → trigger_mode 'once'", () => {
    expect(deriveTriggerMode("button_once")).toBe("once");
});
test("joystick action type → trigger_mode 'once'", () => {
    expect(deriveTriggerMode("joystick")).toBe("once");
});
test("undefined action type → trigger_mode 'once'", () => {
    expect(deriveTriggerMode(undefined)).toBe("once");
});

// ── _buildActionOptions-equivalent logic ──────────────────────────────────

console.log("\n_buildActionOptions");

function _buildActionOptions(actionList, selected, isRobot, robotActionsError) {
    if (!actionList.length) {
        if (isRobot && robotActionsError) {
            return `<option value="">⚠ Could not reach robot: ${escHtml(robotActionsError)}</option>`;
        }
        return '<option value="">-- no actions available --</option>';
    }
    let html = '<option value="">-- Action --</option>';
    for (const a of actionList) {
        const lbl = a.type === "button_hold" ? " (Hold)" : "";
        html += `<option value="${escHtml(a.name)}"${a.name === selected ? " selected" : ""}>${escHtml(a.name)}${lbl}</option>`;
    }
    return html;
}

test("empty list with no error → no-actions placeholder", () => {
    const html = _buildActionOptions([], "", false, "");
    expect(html).toContain("no actions available");
});
test("empty list with robot error → error message shown", () => {
    const html = _buildActionOptions([], "", true, "Timeout");
    expect(html).toContain("Timeout");
    expect(html).toContain("Could not reach robot");
});
test("robot error message is escaped", () => {
    const html = _buildActionOptions([], "", true, "<script>alert(1)</script>");
    expect(html).notToContain("<script>");
    expect(html).toContain("&lt;script&gt;");
});
test("non-empty list renders options", () => {
    const actions = [
        { name: "Drive",  type: "joystick" },
        { name: "Honk",   type: "button_once" },
        { name: "Boost",  type: "button_hold" },
    ];
    const html = _buildActionOptions(actions, "Drive", false, "");
    expect(html).toContain(`value="Drive"`);
    expect(html).toContain("selected");
    expect(html).toContain("(Hold)");
    expect(html).notToContain("no actions available");
});
test("action name with special chars is escaped in option", () => {
    const actions = [{ name: `<Evil "Action">`, type: "button_once" }];
    const html = _buildActionOptions(actions, "", false, "");
    expect(html).notToContain("<Evil");
    expect(html).toContain("&lt;Evil");
});
test("unselected action has no selected attribute", () => {
    const actions = [{ name: "A", type: "button_once" }, { name: "B", type: "button_once" }];
    const html = _buildActionOptions(actions, "A", false, "");
    // Only one 'selected' in the output
    const count = (html.match(/selected/g) || []).length;
    expect(count).toBe(1);
});

// ── Summary ────────────────────────────────────────────────────────────────

console.log(`\n${_passed + _failed} tests: ${_passed} passed, ${_failed} failed\n`);
process.exit(_failed > 0 ? 1 : 0);
